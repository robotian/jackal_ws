#!/usr/bin/env python3

# Copyright 2024 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# We need numpy for vector/matrix math
import numpy as np
# We use tf_transformations for quaternion-to-matrix conversion
# This is a common ROS library.
# If not installed, run: sudo apt install ros-humble-tf-transformations
try:
    from tf_transformations import quaternion_inverse, quaternion_matrix
except ImportError:
    # Log an error and exit if the library is not found
    print("**************************************************************")
    print("Failed to import 'tf_transformations'.")
    print("This node requires this library for quaternion math.")
    print("Please install it by running:")
    print("  sudo apt install ros-humble-tf-transformations")
    print("**************************************************************")
    import sys
    sys.exit(1)


class OdomTwistExtractor(Node):
    """
    A ROS 2 node that subscribes to an Odometry topic,
    extracts the Twist data, and republishes it on a new topic.
    """
    def __init__(self):
        super().__init__('odom_twist_repub_node')
        
        # --- Parameters ---
        # Declare parameters for topic names
        # self.declare_parameter('odom_topic', '/j100_0921/rigidbody_1/odom')
        # self.declare_parameter('twist_topic', '/base_link_twist_map')

        self.declare_parameter('odom_topic', '/j100_0921/odom_converted')
        self.declare_parameter('twist_topic', '/base_link_twist_odom')
        
        # Get topic names from parameters
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        twist_topic = self.get_parameter('twist_topic').get_parameter_value().string_value
        
        # --- Subscriber ---
        # Subscribes to the input odometry topic
        self.odom_subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10) # QoS profile depth
        
        # --- Publisher ---
        # Publishes the extracted linear and angular velocities
        self.twist_publisher = self.create_publisher(
            Twist,
            twist_topic,
            10) # QoS profile depth

        self.get_logger().info(
            f"Node initialized. Subscribing to '{odom_topic}' and "
            f"publishing to '{twist_topic}'."
        )

    def odom_callback(self, msg: Odometry):
        """
        Callback function for the odometry subscriber.
        This function receives an Odometry message, transforms the Twist
        from the 'map' frame to the 'base_link' frame, and publishes it.
        """
        
        # As per your request:
        # We assume msg.twist.twist (linear and angular velocities) is
        # expressed in the 'map' frame (msg.header.frame_id).
        # We need to convert it to the 'base_link' frame (msg.child_frame_id).

        # 1. Get the orientation of base_link relative to map
        # This quaternion (q_bl_to_map) represents the rotation from base_link to map
        q_bl_to_map = msg.pose.pose.orientation
        
        # 2. Get the inverse orientation (map relative to base_link)
        # This quaternion (q_map_to_bl) represents the rotation from map to base_link
        q_map_to_bl_list = quaternion_inverse([
            q_bl_to_map.x,
            q_bl_to_map.y,
            q_bl_to_map.z,
            q_bl_to_map.w
        ])

        # 3. Convert this inverse quaternion to a 4x4 transformation matrix
        # We only care about the 3x3 rotation part (R_map_to_bl)
        transform_matrix = quaternion_matrix(q_map_to_bl_list)
        rotation_matrix = transform_matrix[0:3, 0:3]

        # 4. Get the velocity vectors expressed in the 'map' frame
        v_map = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        
        w_map = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ])

        # 5. Rotate the velocity vectors from 'map' frame to 'base_link' frame
        # v_base = R_map_to_base * v_map
        # w_base = R_map_to_base * w_map
        v_base = rotation_matrix.dot(v_map)
        w_base = rotation_matrix.dot(w_map)

        # 6. Create the new Twist message with transformed velocities
        twist_msg_base_link = Twist()
        
        twist_msg_base_link.linear.x = v_base[0]
        twist_msg_base_link.linear.y = v_base[1]
        twist_msg_base_link.linear.z = v_base[2]
        
        twist_msg_base_link.angular.x = w_base[0]
        twist_msg_base_link.angular.y = w_base[1]
        twist_msg_base_link.angular.z = w_base[2]
        
        # Log the velocities (optional, can be noisy)
        # self.get_logger().info(
        #     f"Publishing Twist: Linear(x={twist_msg_base_link.linear.x:.2f}, "
        #     f"y={twist_msg_base_link.linear.y:.2f}), "
        #     f"Angular(z={twist_msg_base_link.angular.z:.2f})"
        # )

        # Publish the transformed Twist message
        self.twist_publisher.publish(twist_msg_base_link)

def main(args=None):
    """
    Main function to initialize and run the ROS 2 node.
    """
    rclpy.init(args=args)
    
    odom_twist_extractor_node = OdomTwistExtractor()
    
    try:
        rclpy.spin(odom_twist_extractor_node)
    except KeyboardInterrupt:
        # Handle Ctrl+C shutdown
        pass
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector cleans up the node object)
        odom_twist_extractor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


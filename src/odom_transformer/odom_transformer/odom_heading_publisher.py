# ROS 2 Node: odom_heading_tf_publisher_param

# Description:
# ------------
# This node computes heading (yaw) from two GPS-based UTM odometry sources and
# broadcasts a static transform between `map` and `odom` frames using that heading.

# It assumes the front and rear GPS receivers publish `nav_msgs/Odometry` messages
# with positions in the UTM coordinate system. By calculating the vector from the
# rear to the front, the node computes the yaw (heading) and broadcasts the transform
# as a rotation from `map` to `odom`.

# Use this node to initialize the robot's global orientation based on a dual-GPS
# baseline instead of relying on IMU or odometry yaw estimation.

# Configuration:
# --------------
# This node is fully configurable via ROS 2 parameters:

# - `front_odom_topic`: ROS topic for front GPS odometry (default: /gps/front/odom)
# - `rear_odom_topic` : ROS topic for rear GPS odometry (default: /gps/rear/odom)
# - `parent_frame_id` : Frame ID of the parent frame (default: map)
# - `child_frame_id`  : Frame ID of the child frame (default: odom)

# Dependencies:
# -------------
# - rclpy
# - nav_msgs.msg
# - geometry_msgs.msg
# - tf2_ros
# - scipy.spatial.transform.Rotation (recommended over tf_transformations)
# - math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
import math

class OdomHeadingTFPublisher(Node):
    def __init__(self):
        super().__init__('odom_heading_tf_publisher_param')

        # Declare and get parameters
        self.declare_parameter('front_odom_topic', '/gps/front/odom')
        self.declare_parameter('rear_odom_topic', '/gps/rear/odom')
        self.declare_parameter('parent_frame_id', 'map')
        self.declare_parameter('child_frame_id', 'odom')

        self.front_topic = self.get_parameter('front_odom_topic').get_parameter_value().string_value
        self.rear_topic = self.get_parameter('rear_odom_topic').get_parameter_value().string_value
        self.parent_frame = self.get_parameter('parent_frame_id').get_parameter_value().string_value
        self.child_frame = self.get_parameter('child_frame_id').get_parameter_value().string_value

        self.get_logger().info(f"Subscribing to: {self.front_topic}, {self.rear_topic}")
        self.get_logger().info(f"Publishing TF: {self.parent_frame} → {self.child_frame}")

        # Subscribers
        self.front_odom = None
        self.rear_odom = None

        self.front_sub = self.create_subscription(Odometry, self.front_topic, self.front_cb, 10)
        self.rear_sub = self.create_subscription(Odometry, self.rear_topic, self.rear_cb, 10)

        self.tf_broadcaster = TransformBroadcaster(self)

    def front_cb(self, msg):
        self.front_odom = msg
        self.publish_tf()

    def rear_cb(self, msg):
        self.rear_odom = msg
        self.publish_tf()

    def publish_tf(self):
        if self.front_odom is None or self.rear_odom is None:
            return

        dx = self.front_odom.pose.pose.position.x - self.rear_odom.pose.pose.position.x
        dy = self.front_odom.pose.pose.position.y - self.rear_odom.pose.pose.position.y
        heading = math.atan2(dy, dx)
        q = R.from_euler('z', heading).as_quat()
        # q = quaternion_from_euler(0.0, 0.0, heading)
        # q = {0, 0, 0, 1}
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomHeadingTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()

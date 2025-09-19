# #!/usr/bin/env python3
# """
# Node: dual_gps_heading_odom_publisher

# Description:
# ------------
# Computes heading from two UTM-based Odometry messages (e.g., front and rear GPS) and
# publishes a static `nav_msgs/Odometry` message (with orientation only) for use in an
# EKF. Intended to run inside a namespace.

# The output odometry will reflect the node's namespace (e.g., `/my_robot/dual_gps/heading_odom`).

# Parameters:
# -----------
# - front_odom_topic (str): topic for front GPS Odometry (default: '/gps/front/odom')
# - rear_odom_topic  (str): topic for rear GPS Odometry (default: '/gps/rear/odom')
# - frame_id         (str): frame of the published Odometry (e.g., 'map')
# - child_frame_id   (str): child frame (e.g., 'odom')

# Author:
# -------
# OpenAI ChatGPT, 2025
# """

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Quaternion
# from scipy.spatial.transform import Rotation as R
# import math

# class DualGPSHeadingOdomPublisher(Node):
#     def __init__(self):
#         super().__init__('dual_gps_heading_odom_publisher')

#         # Parameters
#         self.declare_parameter('front_odom_topic', '/gps/front/odom')
#         self.declare_parameter('rear_odom_topic', '/gps/rear/odom')
#         self.declare_parameter('frame_id', 'map')
#         self.declare_parameter('child_frame_id', 'odom')

#         self.front_topic = self.get_parameter('front_odom_topic').get_parameter_value().string_value
#         self.rear_topic = self.get_parameter('rear_odom_topic').get_parameter_value().string_value
#         self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
#         self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value

#         self.front_odom = None
#         self.rear_odom = None

#         self.create_subscription(Odometry, self.front_topic, self.front_cb, 10)
#         self.create_subscription(Odometry, self.rear_topic, self.rear_cb, 10)

#         # Publisher with namespace-aware topic
#         self.odom_pub = self.create_publisher(Odometry, 'heading_odom', 10)

#         self.get_logger().info(f"[{self.get_namespace()}] Publishing heading odometry on 'heading_odom'")
#         self.get_logger().info(f"Front topic: {self.front_topic}, Rear topic: {self.rear_topic}")

#     def front_cb(self, msg):
#         self.front_odom = msg
#         self.publish_heading_odom()

#     def rear_cb(self, msg):
#         self.rear_odom = msg
#         self.publish_heading_odom()

#     def publish_heading_odom(self):
#         if self.front_odom is None or self.rear_odom is None:
#             return

#         # Compute heading from front to rear
#         dx = self.front_odom.pose.pose.position.x - self.rear_odom.pose.pose.position.x
#         dy = self.front_odom.pose.pose.position.y - self.rear_odom.pose.pose.position.y
#         heading = math.atan2(dy, dx)
#         q = R.from_euler('z', heading).as_quat()

#         odom = Odometry()
#         odom.header.stamp = self.get_clock().now().to_msg()
#         odom.header.frame_id = self.frame_id
#         odom.child_frame_id = self.child_frame_id

#         odom.pose.pose.position.x = 0.0
#         odom.pose.pose.position.y = 0.0
#         odom.pose.pose.position.z = 0.0
#         odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

#         odom.pose.covariance = [
#             1e-9, 0,    0,    0, 0, 0,
#             0,    1e-9, 0,    0, 0, 0,
#             0,    0,    1e-3, 0, 0, 0,
#             0,    0,    0,    1e-3, 0, 0,
#             0,    0,    0,    0, 1e-3, 0,
#             0,    0,    0,    0, 0, 0.01  # yaw covariance
#         ]

#         self.odom_pub.publish(odom)

# def main(args=None):
#     rclpy.init(args=args)
#     node = DualGPSHeadingOdomPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()



#!/usr/bin/env python3
"""
Node: dual_gps_heading_odom_publisher

Description:
------------
Computes heading from two UTM-based Odometry messages and publishes an Odometry
message with only orientation (yaw) for use in robot_localization's EKF.

Includes timeout checks and warnings if either input topic is not being published.

Author: OpenAI ChatGPT, 2025
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from scipy.spatial.transform import Rotation as R
import math

class DualGPSHeadingOdomPublisher(Node):
    def __init__(self):
        super().__init__('dual_gps_heading_odom_publisher')

        # Parameters
        self.declare_parameter('front_odom_topic', '/gps/front/odom')
        self.declare_parameter('rear_odom_topic', '/gps/rear/odom')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'odom')

        self.front_topic = self.get_parameter('front_odom_topic').get_parameter_value().string_value
        self.rear_topic = self.get_parameter('rear_odom_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value

        self.front_odom = None
        self.rear_odom = None
        self.last_front_time = None
        self.last_rear_time = None
        self.timeout_sec = 3.0

        # Subscribers
        self.create_subscription(Odometry, self.front_topic, self.front_cb, 30)
        self.create_subscription(Odometry, self.rear_topic, self.rear_cb, 30)

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, 'heading_odom', 30)

        # Timer to check topic activity
        self.timer = self.create_timer(1.0, self.check_topic_activity)
        self.front_warned = False
        self.rear_warned = False

        self.get_logger().info(f"[{self.get_namespace()}] Publishing heading odometry on 'heading_odom'")
        self.get_logger().info(f"Front topic: {self.front_topic}, Rear topic: {self.rear_topic}")

    def front_cb(self, msg):
        self.front_odom = msg
        self.last_front_time = self.get_clock().now()
        self.front_warned = False
        self.publish_heading_odom()

    def rear_cb(self, msg):
        self.rear_odom = msg
        self.last_rear_time = self.get_clock().now()
        self.rear_warned = False
        self.publish_heading_odom()

    def publish_heading_odom(self):
        if self.front_odom is None or self.rear_odom is None:
            return

        dx = self.front_odom.pose.pose.position.x - self.rear_odom.pose.pose.position.x
        dy = self.front_odom.pose.pose.position.y - self.rear_odom.pose.pose.position.y
        heading = math.atan2(dy, dx)
        q = R.from_euler('z', heading).as_quat()

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = self.front_odom.pose.pose.position.x # fake
        odom.pose.pose.position.y = self.front_odom.pose.pose.position.y # fake
        odom.pose.pose.position.z = self.front_odom.pose.pose.position.z # fake
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        odom.pose.covariance = [
            1e-9, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1e-9, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e-3, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e-3, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e-3, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01
        ]   

        self.odom_pub.publish(odom)

    def check_topic_activity(self):
        now = self.get_clock().now()

        if self.last_front_time is None or (now - self.last_front_time).nanoseconds > self.timeout_sec * 1e9:
            if not self.front_warned:
                self.get_logger().warn(f"No messages received on front odometry topic [{self.front_topic}] in the last {self.timeout_sec} seconds")
                self.front_warned = True

        if self.last_rear_time is None or (now - self.last_rear_time).nanoseconds > self.timeout_sec * 1e9:
            if not self.rear_warned:
                self.get_logger().warn(f"No messages received on rear odometry topic [{self.rear_topic}] in the last {self.timeout_sec} seconds")
                self.rear_warned = True

def main(args=None):
    rclpy.init(args=args)
    node = DualGPSHeadingOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

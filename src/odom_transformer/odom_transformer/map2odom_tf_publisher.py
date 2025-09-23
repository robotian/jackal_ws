import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_from_matrix

import numpy as np
# from scipy.spatial.transform import Rotation as R


def quat_mult(q1, q2):
    # Quaternion multiplication
    w1, x1, y1, z1 = q1[3], q1[0], q1[1], q1[2]
    w2, x2, y2, z2 = q2[3], q2[0], q2[1], q2[2]
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return [x, y, z, w]

def pose_to_homogeneous_matrix(pose):
    # Extract position
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z
    # Extract orientation as quaternion
    qx = pose.orientation.x
    qy = pose.orientation.y
    qz = pose.orientation.z
    qw = pose.orientation.w

    # Compute 3x3 rotation matrix from quaternion
    R = np.array([
        [1 - 2*qy**2 - 2*qz**2,     2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw,     1 - 2*qx**2 - 2*qz**2,     2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw,         2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
    ])

    # Construct homogeneous transformation matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.array([x, y, z])
    return T

class MapToOdomTf(Node):
    def __init__(self):
        super().__init__('map2odom_tf_publisher')
        self.odom_msg = None
        self.pose_msg = None

        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Odometry, 'odom_converted', self.odom_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'rigidbody_1/pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.05, self.publish_tf)

    def odom_callback(self, msg):
        self.odom_msg = msg

    def pose_callback(self, msg):
        self.pose_msg = msg

    def publish_tf(self):
        if self.odom_msg is None or self.pose_msg is None:
            return

        # The transform from 'map' to 'odom' is the difference between the robotpose
        # (in map frame) and odometry pose (in map frame)
        pose_map = self.pose_msg.pose.pose
        pose_odom = self.odom_msg.pose.pose

        # Calculate the transform: pose_map - pose_odom
        trans = TransformStamped()
        trans.header.stamp = self.get_clock().now().to_msg()
        trans.header.frame_id = 'map'
        trans.child_frame_id = 'odom'

        T_m_b = pose_to_homogeneous_matrix(pose_map)   # map to baselink
        T_o_b = pose_to_homogeneous_matrix(pose_odom)  # odom to baselink
        # Inverse of T_o_b gives T_b_o (baselink to odom)
        # Therefore, T_m_o = T_m_b * inverse(T_b_o )
        T_m_o = T_m_b @ np.linalg.inv(T_o_b)


        # Position difference
        trans.transform.translation.x = T_m_o[0, 3]
        trans.transform.translation.y = T_m_o[1, 3]
        trans.transform.translation.z = T_m_o[2, 3]

        quat = quaternion_from_matrix(T_m_o)

        trans.transform.rotation.x = quat[0]
        trans.transform.rotation.y = quat[1]
        trans.transform.rotation.z = quat[2]
        trans.transform.rotation.w = quat[3]

        # Position difference
        # trans.transform.translation.x = pose_map.position.x - pose_odom.position.x
        # trans.transform.translation.y = pose_map.position.y - pose_odom.position.y
        # trans.transform.translation.z = pose_map.position.z - pose_odom.position.z

        # Orientation difference (by quaternion multiplication: q_map * inverse(q_odom))
        # q_map = [pose_map.orientation.x, pose_map.orientation.y,
        #          pose_map.orientation.z, pose_map.orientation.w]
        # q_odom = [pose_odom.orientation.x, pose_odom.orientation.y,
        #           pose_odom.orientation.z, pose_odom.orientation.w]

        # # Compute inverse of odom quaternion
        # q_odom_inv = [ -q_odom[0], -q_odom[1], -q_odom[2],  q_odom[3] ]
        # # Multiply: q_map * q_odom_inv
        # # This produces the rotation from odom to map, which must be extracted appropriately.
        

        # q_res = quat_mult(q_map, q_odom_inv)
        # trans.transform.rotation.x = q_res[0]
        # trans.transform.rotation.y = q_res[1]
        # trans.transform.rotation.z = q_res[2]
        # trans.transform.rotation.w = q_res[3]

        self.tf_broadcaster.sendTransform(trans)

def main(args=None):
    rclpy.init(args=args)
    node = MapToOdomTf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

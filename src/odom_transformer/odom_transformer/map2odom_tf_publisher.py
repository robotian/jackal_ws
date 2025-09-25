import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster
# from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_from_matrix
# import quaternion
import math
import numpy as np
# from scipy.spatial.transform import Rotation as R

def rotation_matrix_to_quaternion(R):
    """
    Convert a 3x3 rotation matrix to a quaternion (w, x, y, z).

    Args:
        R: 3x3 rotation matrix as a list of lists or nested lists.

    Returns:
        A tuple representing the quaternion (w, x, y, z).
    """
    r11, r12, r13 = R[0][0], R[0][1], R[0][2]
    r21, r22, r23 = R[1][0], R[1][1], R[1][2]
    r31, r32, r33 = R[2][0], R[2][1], R[2][2]

    trace = r11 + r22 + r33

    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2  # s=4*qw
        qw = 0.25 * s
        qx = (r32 - r23) / s
        qy = (r13 - r31) / s
        qz = (r21 - r12) / s
    elif (r11 > r22) and (r11 > r33):
        s = math.sqrt(1.0 + r11 - r22 - r33) * 2  # s=4*qx
        qw = (r32 - r23) / s
        qx = 0.25 * s
        qy = (r12 + r21) / s
        qz = (r13 + r31) / s
    elif r22 > r33:
        s = math.sqrt(1.0 + r22 - r11 - r33) * 2  # s=4*qy
        qw = (r13 - r31) / s
        qx = (r12 + r21) / s
        qy = 0.25 * s
        qz = (r23 + r32) / s
    else:
        s = math.sqrt(1.0 + r33 - r11 - r22) * 2  # s=4*qz
        qw = (r21 - r12) / s
        qx = (r13 + r31) / s
        qy = (r23 + r32) / s
        qz = 0.25 * s

    return (qx, qy, qz, qw)

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


def inverse_homogeneous_matrix(T):
    """
    Compute the inverse of a 4x4 homogeneous transformation matrix.

    Args:
        T: 4x4 nested list or 2D list representing the homogeneous matrix.

    Returns:
        4x4 nested list: Inverse homogeneous matrix.
    """
    # Extract rotation and translation
    R = [row[:3] for row in T[:3]]  # 3x3 rotation
    t = [T[i][3] for i in range(3)]  # 3x1 translation

    # Transpose of rotation matrix (inverse of rotation)
    R_inv = [[R[j][i] for j in range(3)] for i in range(3)]

    # Compute -R^T * t
    t_inv = [
        -sum(R_inv[i][j] * t[j] for j in range(3))
        for i in range(3)
    ]

    # Construct inverse homogeneous matrix
    T_inv = [R_inv[0] + [t_inv[0]],
             R_inv[1] + [t_inv[1]],
             R_inv[2] + [t_inv[2]],
             [0, 0, 0, 1]]

    return T_inv

class MapToOdomTf(Node):
    def __init__(self):
        super().__init__('map2odom_tf_publisher')
        self.odom_msg = None
        self.pose_msg = None

        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Odometry, 'odom_converted', self.odom_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'rigidbody_1/pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.1, self.publish_tf)

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
        trans.header.stamp = self.get_clock().now().to_msg() # Use current time
        trans.header.frame_id = 'map'
        trans.child_frame_id = 'odom'

        T_m_b = pose_to_homogeneous_matrix(pose_map)   # map to baselink
        T_o_b = pose_to_homogeneous_matrix(pose_odom)  # odom to baselink
        # Inverse of T_o_b gives T_b_o (baselink to odom)
        # Therefore, T_m_o = T_m_b * inverse(T_b_o )
        # T_m_o = T_m_b @ np.linalg.inv(T_o_b)
        T_m_o = T_m_b @ inverse_homogeneous_matrix(T_o_b)
        

        # dummy T_m_o
        # T_m_o = np.eye(4)


        # # Position difference
        trans.transform.translation.x = T_m_o[0, 3]
        trans.transform.translation.y = T_m_o[1, 3]
        trans.transform.translation.z = T_m_o[2, 3]

    
        quat = rotation_matrix_to_quaternion(T_m_o)

        # quat = np.array([0, 0, 0, 1])  # dummy quat

        trans.transform.rotation.x = quat[0]
        trans.transform.rotation.y = quat[1]
        trans.transform.rotation.z = quat[2]
        trans.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(trans)

def main(args=None):
    rclpy.init(args=args)
    node = MapToOdomTf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

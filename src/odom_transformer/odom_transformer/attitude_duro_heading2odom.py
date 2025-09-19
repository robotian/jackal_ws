import rclpy
from rclpy.node import Node
from swiftnav_ros2_driver.msg import Baseline
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
# import tf_transformations
import math

def normalize_quaternion(q: Quaternion) -> Quaternion:
    """Normalizes a geometry_msgs.msg.Quaternion in place."""
    norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
    if norm == 0 or math.isnan(norm):
        # Fallback to identity quaternion if invalid
        q.x, q.y, q.z, q.w = 0.0, 0.0, 0.0, 1.0
    else:
        q.x /= norm
        q.y /= norm
        q.z /= norm
        q.w /= norm
    return q

def yaw_to_quaternion(yaw):
    """Returns quaternion for a given yaw (in radians)."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return normalize_quaternion(q)

# def yaw_to_quaternion(yaw):
#     # Returns quaternion for a given yaw (in radians)
#     q = Quaternion()
#     q.x = 0.0
#     q.y = 0.0
#     q.z = math.sin(yaw / 2.0)
#     q.w = math.cos(yaw / 2.0)
#     return q

class HeadingToOdometry(Node):
    def __init__(self):
        super().__init__('attitude_duro_heading2odom')
        self.subscription = self.create_subscription(
            Baseline,
            'sensors/gps_1/baseline',  # attitude gps
            self.baseline_callback,
            10)
        self.publisher = self.create_publisher(Odometry, 'heading_odom', 10)

    def baseline_callback(self, msg):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'

        # Set default position to origin
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0

        # Assuming 'heading' field in Baseline is degrees; convert to radians
        # Replace 'msg.heading' with actual heading field name from Baseline msg
        heading_degrees = -msg.baseline_dir_deg
        heading_radians = math.radians(heading_degrees)
        odom.pose.pose.orientation = yaw_to_quaternion(heading_radians)
        odom.pose.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]  

        # Optionally fill velocity or covariance here if desired

        self.publisher.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = HeadingToOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

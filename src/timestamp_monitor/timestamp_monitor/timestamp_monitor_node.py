import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
import importlib

class HeaderTimestampPrinter(Node):
    def __init__(self):
        super().__init__('timestamp_monitor_node')

        # Declare and get the parameter (topics)
        self.declare_parameter('topics', [ '/j100_0921/sensors/camera_0/stereolabs_zed/point_cloud/cloud_registered'])
        topics = self.get_parameter('topics').get_parameter_value().string_array_value
        self.get_logger().info(f'Monitoring topics: {topics}')

        # Declare and get the parameter (message types)
        self.declare_parameter('msg_types', ['sensor_msgs/msg/PointCloud2'])
        msg_types = self.get_parameter('msg_types').get_parameter_value().string_array_value

        if len(topics) != len(msg_types):
            self.get_logger().error('topics and msg_types parameter lists must have the same length')
            return

        self._subscriptions = []
        for topic, msg_type_str in zip(topics, msg_types):
            self.get_logger().info(f'Subscribing to topic: {topic} with message type: {msg_type_str}')
            # Dynamically import message type
            pkg_name, msg_type, type_name = msg_type_str.split('/')
            msg_module = importlib.import_module(f'{pkg_name}.msg')
            msg_type = getattr(msg_module, type_name)

            def make_callback(topic_name):
                def callback(msg):
                    # Assumes message contains a 'header' attribute of type std_msgs/msg/Header
                    timestamp = getattr(msg.header, 'stamp', None)
                    if timestamp:
                        self.get_logger().info(f'{topic_name} header.stamp: {timestamp.sec}.{timestamp.nanosec}')
                    else:
                        self.get_logger().warn(f'{topic_name} does not have header.stamp')
                return callback

            sub = self.create_subscription(
                msg_type, topic, make_callback(topic), 10)
            self._subscriptions.append(sub)

def main(args=None):
    rclpy.init(args=args)
    node = HeaderTimestampPrinter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

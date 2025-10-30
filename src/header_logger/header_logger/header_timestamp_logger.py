import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import importlib  # Used for dynamic message type importing
import functools  # Used to pass extra arguments to the callback
# from rcl_interfaces.msg import ParameterDescriptor, ParameterType

class HeaderLoggerNode(Node):
    """
    A node that subscribes to a list of topics defined in a param file.
    It dynamically discovers the message types and subscribes.
    It then prints the 'header.stamp' of received messages.
    """

    def __init__(self):
        super().__init__('header_timestamp_logger')
        self.get_logger().info(f"Node initialized: {self.get_name()}")

        # Declare the string array parameter
        # string_array_descriptor = ParameterDescriptor(
        #         type=ParameterType.PARAMETER_STRING_ARRAY,
        #         description='A list of strings.'
        #     )

        # Declare and get the list of topic names from the parameters
        self.declare_parameter('topic_names', ['/j100_0921/sensors/imu_0/data'])
        # self.topic_list = self.get_parameter('topic_names').value

        self.topic_list = self.get_parameter('topic_names').get_parameter_value().string_array_value


        if not self.topic_list:
            self.get_logger().warn("No topics specified in 'topic_names' parameter. Node will do nothing.")
            return

        self.get_logger().info(f"Monitoring topics: {self.topic_list}")
        

        # This dictionary will hold our active subscription objects,
        # mapping a topic name to its subscriber.
        self.subscriptions = {}
        self.get_logger().info(f"Active topics check...")
        # We'll use a timer to periodically check for new topics
        # that match our list. This allows the node to be started
        # before the topics it needs to monitor are available.
        
        self.check_topics_timer = self.create_timer(2.0, self.discover_and_subscribe)

    def discover_and_subscribe(self):
        """
        Timer callback. Checks for active topics, compares against our
        param list, and subscribes to new, matching topics.
        """


        
        # Get a list of all currently active topics and their types
        active_topics = self.get_topic_names_and_types()

        # Create a clean map of topic_name -> topic_type
        # We take [0] because get_topic_names_and_types() returns a list of types
        # for each topic (in case of multiple publishers with different QOS, etc.)
        active_topic_map = {name: info[0] for name, info in active_topics}
        for topic_name in self.topic_list:
            # Check if we are already subscribed
            if topic_name in self.subscriptions:
                continue

            # Check if this desired topic is now active
            if topic_name in active_topic_map:
                topic_type_str = active_topic_map[topic_name]

                self.get_logger().info(f"Found topic '{topic_name}' with type '{topic_type_str}'. Attempting to subscribe.")

                try:
                    # Dynamically import the message type
                    msg_type = self.import_message_type(topic_type_str)

                    # Create a specific callback for this topic
                    # We use functools.partial to pass the topic_name
                    # to the generic_callback.
                    callback_with_topic_name = functools.partial(
                        self.generic_callback, topic_name=topic_name
                    )

                    # Create the subscription
                    sub = self.create_subscription(
                        msg_type,
                        topic_name,
                        callback_with_topic_name,
                        10  # QoS depth
                    )

                    # Store the subscription object to keep it alive
                    # and to track that we are subscribed.
                    self.subscriptions[topic_name] = sub

                except Exception as e:
                    self.get_logger().error(f"Failed to subscribe to '{topic_name}': {e}")

    def import_message_type(self, type_str):
        """
        Dynamically imports a ROS message type from its string representation,
        e.g., 'sensor_msgs/msg/Image'.
        """
        # Split the string 'sensor_msgs/msg/Image'
        parts = type_str.split('/')
        if len(parts) != 3:
            raise ValueError(f"Invalid message type string: {type_str}")

        # module_name = 'sensor_msgs.msg'
        module_name = f"{parts[0]}.{parts[1]}"
        # msg_name = 'Image'
        msg_name = parts[2]

        # Import the module
        module = importlib.import_module(module_name)
        
        # Get the message class from the module
        return getattr(module, msg_name)

    def generic_callback(self, msg, topic_name):
        """
        This is the generic callback for all subscriptions.
        It attempts to read msg.header.stamp.
        """
        try:
            # The core logic: access the header.stamp
            stamp = msg.header.stamp
            self.get_logger().info(f"[{topic_name}] Timestamp: {stamp.sec}.{stamp.nanosec:09d}")

        except AttributeError:
            # This message type doesn't have a 'header' or 'header.stamp'
            self.get_logger().warn(
                f"Message on topic '{topic_name}' has no 'header.stamp' attribute. "
                "This topic will be ignored."
            )
            
            # Unsubscribe to save resources and stop this warning
            if topic_name in self.subscriptions:
                self.subscriptions[topic_name].destroy()
                del self.subscriptions[topic_name]
        except Exception as e:
            self.get_logger().error(f"Error in callback for '{topic_name}': {e}")


def main(args=None):
    rclpy.init(args=args)
    node = HeaderLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

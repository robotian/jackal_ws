import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped

class GoalVisualizer(Node):
    def __init__(self, namespace: str):
        super().__init__("goal_visualizer")
        self.namespace = namespace
        self.marker_pub = self.create_publisher(MarkerArray, f"{self.namespace}/goal_markers", 10)
        self.get_logger().info("GoalVisualizer initialized — publishing to /goal_markers")

    def publish_pose_array(self, poses: list[PoseStamped], label_prefix="Goal", color=(0.0, 1.0, 0.0)):
        """
        Publishes a MarkerArray of all goal poses for visualization in RViz.

        Args:
            poses (list[PoseStamped]): The list of goal poses.
            label_prefix (str): Prefix for text labels (default: 'Goal').
            color (tuple): (r, g, b) color for the arrows.
        """
        marker_array = MarkerArray()
        marker_array.markers = []
        arrow_scale = (0.3, 0.1, 0.1)
        text_scale = 0.25
        z_offset = 0.2

        for i, pose_stamped in enumerate(poses):
            pose = pose_stamped.pose

            # Arrow marker for each goal
            arrow_marker = Marker()
            arrow_marker.header.frame_id = pose_stamped.header.frame_id or "map"
            arrow_marker.header.stamp = self.get_clock().now().to_msg()
            arrow_marker.ns = "through_goals"
            arrow_marker.id = i
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.pose = pose
            arrow_marker.scale.x, arrow_marker.scale.y, arrow_marker.scale.z = arrow_scale
            arrow_marker.color.a = 1.0
            arrow_marker.color.r, arrow_marker.color.g, arrow_marker.color.b = color

            # Text marker label above arrow
            text_marker = Marker()
            text_marker.header.frame_id = arrow_marker.header.frame_id
            text_marker.header.stamp = arrow_marker.header.stamp
            text_marker.ns = "goal_labels"
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = pose.position.x
            text_marker.pose.position.y = pose.position.y
            text_marker.pose.position.z = pose.position.z + z_offset
            text_marker.scale.z = text_scale
            text_marker.color.a = 1.0
            text_marker.color.r, text_marker.color.g, text_marker.color.b = (1.0, 1.0, 1.0)
            text_marker.text = f"{label_prefix} {i+1}"

            marker_array.markers.append(arrow_marker)
            marker_array.markers.append(text_marker)

        # Clear old markers if needed
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.insert(0, clear_marker)

        self.marker_pub.publish(marker_array)
        self.get_logger().info(f"Published {len(poses)} goal markers for NavigateThroughPoses")

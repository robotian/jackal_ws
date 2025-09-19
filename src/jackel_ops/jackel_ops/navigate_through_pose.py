from dataclasses import asdict
import json
import math
import os
from typing import Optional

from py import log
import action_msgs
import action_msgs.srv
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node
from rclpy.action import ActionClient, client
from rclpy.task import Future

from std_msgs.msg import String
from nav2_msgs.action import NavigateThroughPoses
from nav2_msgs.action._navigate_through_poses import NavigateThroughPoses_FeedbackMessage
from geometry_msgs.msg import PoseStamped, Pose
from status_interfaces.msg import RobotStatus, SubTask, Task, WayPoint

from jackel_ops.enum import RobotStatusEnum
from jackel_ops.dataclass import WPFStatus

logger = RcutilsLogger(os.path.basename(__file__))


class NavigationActionClientUsingNTP:
    def __init__(self,  node: Node) -> None:
        self.node = node

        self.robot_status: RobotStatus
        self.task_status: String
        self.task: Task
        self.sub_task: SubTask
        self.waypoints_list: list[WayPoint] = []
        self.goal_handle: client.ClientGoalHandle
        self.is_goal_cancelled: bool = False

        # Store last recorded waypoint
        self.last_waypoint: Optional[int] = None
        self.last_waypoint_index = 0
        self.current_status = RobotStatusEnum.START_MOVING

        self.namespace = self.node.get_namespace().rstrip('/')

        self.publisher = self.node.create_publisher(
            String, f'{self.namespace}/status/robot/navigation', 10)

        self.status_sub = self.node.create_subscription(
            RobotStatus,
            f'{self.namespace}/status/robot',
            lambda msg: setattr(self, 'robot_status', msg),
            10)

        self.client = ActionClient(
            self.node, NavigateThroughPoses,
            f'{self.namespace}/navigate_through_poses')

    # region CANCEL GOAL
    def cancel_goal(self):
        if self.is_goal_cancelled:
            return

        logger.info('Canceling goal')
        if self.goal_handle is not None:
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(self.goal_canceled_callback)
        else:
            logger.warning("No Navigation Goal to cancel.")
            self.goal_cancelled = True  # Prevent retrying

    def goal_canceled_callback(self, future):
        cancel_response: action_msgs.srv.CancelGoal.Response = future.result()
        logger.info(
            f"Response: {cancel_response.return_code}."
            f" Message Type: {type(cancel_response)}")

        if cancel_response.return_code == 0:
            logger.info('Cancelling of goal complete.')
            self.is_goal_cancelled = True
        else:
            logger.warning('Goal failed to cancel.')
    # endregion

    # region SEND GOAL

    def send_goal(self, task: Task):
        if not task or not task.sub_tasks:
            logger.warning(
                f"Warning while sending Navigation Goal: {task}")
            return

        self.task = task

        st = list(task.sub_tasks)
        self.sub_task = st[0]
        # self.sub_task = SubTask(
        #     **task.sub_tasks) if isinstance(task.sub_tasks, dict) else task.sub_tasks

        if not isinstance(self.sub_task.data, list):
            logger.error("SubTask data must be a list of WayPoints.")
            return

        # self.waypoints_list = self.sub_task.data
        self.waypoints_list = [
            WayPoint(**wp) if isinstance(wp, dict) else wp
            for wp in self.sub_task.data
        ]

        logger.info(f"List of Waypoints: {self.waypoints_list}")

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = self.generate_goal_poses(self.waypoints_list)

        logger.info("Sending navigation goal with waypoints...")

        if not self.client.wait_for_server(timeout_sec=5.0):
            logger.error("NavigateThroughPoses action server not available.")
            return

        # TO DO - implement callback functions
        future = self.client.send_goal_async(
            goal_msg, self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)
    # endregion

    def goal_response_callback(self, future: Future):
        """Handle the response from the action server when a goal is sent."""
        result = future.result()
        if result is not None:
            self.goal_handle = result
        else:
            logger.error("Failed to get a valid goal handle.")
            return

        if not self.goal_handle.accepted:
            logger.error("Goal rejected")
            return
        logger.info("Goal accepted, waiting for result...")

        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future: Future):
        """Process the result of the navigation goal from the action server."""
        result = future.result()
        if not result:
            logger.error("Failed to get a valid result.")
            return

        status = result.status
        # logger.info(
        #     f"Result received. Status: {status}. Message Type:{type(result)}")

        logger.info(
            f"Result received. Status: {status}.")

        if status == 4:
            logger.info("Destination reached successfully.")
            final_wp = self.waypoints_list[-1]
            self.current_status = RobotStatusEnum.DESTINATION_REACHED
            self.publish_status(final_wp.node_id, final_wp.node_id)

        elif status in [5, 6]:  # CANCELED or ABORTED
            current_node_id = (
                self.waypoints_list[self.last_waypoint].node_id
                if self.last_waypoint is not None else -1)

            logger.info(f"Target location: {self.task.target_node_id}")

            msg = ("Goal was canceled."
                   if status == 5 else "Goal was aborted.")
            logger.warning(msg)
            self.current_status = RobotStatusEnum.ERROR
            self.publish_status(
                current_node_id,
                self.task.target_node_id)

        self.last_waypoint = None
        self.last_waypoint_index = 0

    def feedback_callback(self, feedback_msg: NavigateThroughPoses_FeedbackMessage):
        """Handle feedback from the action server during navigation."""
        feedback: NavigateThroughPoses.Feedback = feedback_msg.feedback
        current_pose: Pose = feedback.current_pose.pose
        index = self.get_closest_waypoint_index(current_pose)
        length = len(self.waypoints_list)

        if index != -1 and self.last_waypoint != index:
            logger.info(
                f"Waypoint changed: {self.last_waypoint} → {index}")
            self.last_waypoint = index  # Update last recorded waypoint
            self.current_status = RobotStatusEnum.MOVING

        current_node_id = (
            self.waypoints_list[self.last_waypoint].node_id
            if self.last_waypoint is not None else -1)

        status = WPFStatus(
            status=int(self.current_status.value),
            task=self.sub_task.description if self.sub_task else "No Task",
            current_node_id=current_node_id,
            target_node_id=self.waypoints_list[length-1].node_id,
        )

        if self.last_waypoint is not None:
            status.current_node_id = self.waypoints_list[self.last_waypoint].node_id

        status_json = json.dumps(asdict(status))

        msg = String()
        msg.data = status_json
        self.publisher.publish(msg)

    def publish_status(self, current_node_id: int, target_node_id: int) -> None:
        """
        Publish the current status as a JSON-formatted ROS2 String message.

        Args:
            current_node_id (int or str): The identifier of the current node.
            target_node_id (int or str): The identifier of the target node.

        This method constructs a WPFStatus object with the current status,
        task description, current node ID, and target node ID. It then serializes
        this object to JSON, wraps it in a ROS2 String message, and publishes it
        to the configured topic.
        """
        wpf_status = WPFStatus(
            status=int(self.current_status.value),
            task=self.sub_task.description,
            current_node_id=current_node_id,
            target_node_id=target_node_id
        )

        # logger.info(f"Publishing Status: {wpf_status}")

        status_json = json.dumps(asdict(wpf_status))
        msg = String()
        msg.data = status_json
        self.publisher.publish(msg)

    def generate_goal_poses(self, waypoint_list: list[WayPoint]) -> list[PoseStamped]:
        """
        Generate a list of PoseStamped messages representing goal poses for navigation.

        The first pose corresponds to the robot's current location using ground truth data.
        Subsequent poses are generated from the provided waypoint list. Each pose's orientation
        is calculated to face the direction of the next waypoint, except for the last waypoint,
        which defaults to a neutral orientation.
        Args:
            waypoint_list (list): List of waypoints, where each waypoint has 'node_id', 'x' and 'y'
                attributes representing coordinates.
        Returns:
            list: List of PoseStamped objects with positions and orientations set for navigation.
        """

        waypoint_count = len(waypoint_list)
        poses: list[PoseStamped] = []

        for index, waypoint in enumerate(waypoint_list):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.node.get_clock().now().to_msg()

            # First pose is the robot's current location
            if index == 0:
                pose.pose.position.x = float(
                    self.robot_status.topo_map_position.x)
                pose.pose.position.y = float(
                    self.robot_status.topo_map_position.y)
                pose.pose.orientation.z = -1.57
                pose.pose.orientation.w = 1.0
            else:
                current_wp = WayPoint(
                    **waypoint) if isinstance(waypoint, dict) else waypoint
                next_wp = waypoint_list[index + 1] if index + 1 < waypoint_count else None

                pose.pose.position.x = float(current_wp.x)
                pose.pose.position.y = float(current_wp.y)

                # Pose Orientation calculation pointing towards next pose
                if next_wp and (current_wp.x != 0 or current_wp.y != 0):
                    dx = next_wp.x - current_wp.x
                    dy = next_wp.y - current_wp.y
                    angle = math.atan2(dy, dx)
                    logger.info(f"Angle: {angle}")
                    pose.pose.orientation.z = math.sin(angle / 2)
                    pose.pose.orientation.w = math.cos(angle / 2)
                # Default orientation of pose on last waypoint pose
                else:
                    pose.pose.orientation.z = -1.57
                    pose.pose.orientation.w = 1.0

            poses.append(pose)

        return poses

    def get_closest_waypoint_index(self, current_pose: Pose) -> int:
        """
        Finds the closest waypoint based on distance but ensures switching happens with a buffer.

        This method calculates the squared distance to each waypoint and compares it to the
        squared distance of the current waypoint. It uses a threshold to determine if the
        robot has reached a waypoint. The method also includes a buffer to delay switching
        until the robot is significantly closer to the next waypoint.
        """
        min_dist = float('inf')
        closest_index = self.last_waypoint_index
        threshold = 0.1 ** 2  # 0.1m² area threshold
        switch_buffer = 0.3  # Delay switching until it's 0.3m closer than the current waypoint

        current_wp = self.waypoints_list[self.last_waypoint_index]
        current_dist = (current_wp.x - current_pose.position.x) ** 2 + \
            (current_wp.y - current_pose.position.y) ** 2

        for i, waypoint in enumerate(self.waypoints_list):
            dist_squared = (waypoint.x - current_pose.position.x) ** 2 + \
                (waypoint.y - current_pose.position.y) ** 2

            if dist_squared <= threshold:
                # logger.info(f"Reached waypoint {waypoint.node_id}")
                self.last_waypoint_index = i
                return i

            # Only switch if it's significantly closer than the current waypoint
            if dist_squared < min_dist and dist_squared < (current_dist - switch_buffer ** 2):
                min_dist = dist_squared
                closest_index = i

        self.last_waypoint_index = closest_index
        return closest_index
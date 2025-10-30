from dataclasses import asdict
import json
import math
import os
import time
from transforms3d import euler

import action_msgs
import action_msgs.srv
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node
from rclpy.action import ActionClient, client
from rclpy.task import Future

from geometry_msgs.msg import PoseStamped, Pose
from nav2_msgs.action import NavigateThroughPoses
from nav2_msgs.action._navigate_through_poses import NavigateThroughPoses_FeedbackMessage
from std_msgs.msg import String
from status_interfaces.msg import RobotStatus, SubTask, Task, WayPoint

from jackel_ops.enum import RobotStatusEnum
from jackel_ops.dataclass import WPFStatus

logger = RcutilsLogger(os.path.basename(__file__))


class NavigationActionClientUsingNTP:
    def __init__(self, node: Node) -> None:
        self.node = node

        self.robot_status: RobotStatus
        self.task_status: String
        self.task: Task | None = None
        self.sub_task: SubTask | None = None
        self.waypoints_list: list[WayPoint] = []
        self.goal_handle: client.ClientGoalHandle | None = None
        self.is_goal_cancelled: bool = False
        self.retry_count: int = 0
        self.max_retries: int = 3

        # Store last recorded waypoint
        self.last_waypoint: int | None = None
        self.last_waypoint_index = 0
        self.current_status = RobotStatusEnum.START_MOVING

        # Flag to prevent multiple simultaneous goals
        self.goal_in_progress: bool = False
        self.result_received: bool = False

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

    def cancel_goal(self):
        """Cancel the current navigation goal"""
        if self.is_goal_cancelled:
            logger.info("Goal already cancelled")
            return

        if not self.goal_in_progress:
            logger.info("No goal in progress to cancel")
            return

        logger.info('Canceling navigation goal')
        if self.goal_handle is not None:
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(self.goal_canceled_callback)
        else:
            logger.warning("No goal handle available to cancel.")
            self.is_goal_cancelled = True
            self.goal_in_progress = False

    def goal_canceled_callback(self, future):
        """Handle goal cancellation response"""
        cancel_response: action_msgs.srv.CancelGoal.Response = future.result()
        logger.info(
            f"Cancel response code: {cancel_response.return_code}")

        if cancel_response.return_code == 0:
            logger.info('Goal cancellation complete.')
            self.is_goal_cancelled = True
            self.goal_in_progress = False
        else:
            logger.warning('Goal failed to cancel.')

    def send_goal(self, task: Task):
        """Send navigation goal to action server"""
        # CRITICAL: Prevent sending goal if one is already in progress
        if self.goal_in_progress and not self.result_received:
            logger.warning(
                "Navigation goal already in progress - skipping duplicate send")
            return

        if not task or not task.sub_tasks:
            logger.warning(
                f"Invalid task for navigation: {task}")
            return

        self.task = task
        st = list(task.sub_tasks)
        self.sub_task = st[0]

        if not self.sub_task or not isinstance(self.sub_task.data, list):
            logger.error("SubTask data must be a list of WayPoints.")
            return

        # Parse waypoints
        self.waypoints_list = [
            WayPoint(**wp) if isinstance(wp, dict) else wp
            for wp in self.sub_task.data
        ]

        logger.debug(f"List of Waypoints: {self.waypoints_list}")
        logger.info(f"Retry count: {self.retry_count}/{self.max_retries}")

        # Generate poses
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = self.generate_goal_poses(self.waypoints_list)

        logger.info("Sending navigation goal with waypoints...")

        # Check server availability
        if not self.client.wait_for_server(timeout_sec=5.0):
            logger.error("NavigateThroughPoses action server not available.")

            return

        # Mark goal as in progress
        self.goal_in_progress = True
        self.result_received = False
        self.is_goal_cancelled = False

        # Send goal
        future = self.client.send_goal_async(
            goal_msg, self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future: Future):
        """Handle the response from the action server when a goal is sent."""
        self.goal_handle = future.result()

        if self.goal_handle is None:
            logger.error("Failed to get a valid goal handle.")
            self.goal_in_progress = False
            return

        if not self.goal_handle.accepted:
            logger.error("Goal rejected by action server")
            self.goal_in_progress = False
            return

        logger.info("Goal accepted, waiting for result...")
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future: Future):
        """Process the result of the navigation goal from the action server."""
        result = future.result()

        # logger.info(f"Result: {result}")

        if not result:
            logger.error("Failed to get a valid result.")
            self.goal_in_progress = False
            return

        status = result.status
        self.result_received = True
        self.goal_in_progress = False

        logger.info(f"Result received. Status: {status}")

        if status == 4:  # SUCCEEDED
            logger.info("Destination reached successfully.")
            final_wp = self.waypoints_list[-1]
            self.current_status = RobotStatusEnum.DESTINATION_REACHED
            self.publish_status(final_wp.node_id, final_wp.node_id)

            # Reset for next goal
            self.retry_count = 0
            self.last_waypoint = None
            self.last_waypoint_index = 0

        elif status == 5:  # CANCELED
            logger.warning("Goal was canceled.")
            current_node_id = self._get_current_node_id()
            self.current_status = RobotStatusEnum.ERROR

            if self.task:
                self.publish_status(current_node_id, self.task.target_node_id)

            # Don't retry if goal was explicitly canceled
            self.retry_count = 0
            self.last_waypoint = None
            self.last_waypoint_index = 0

        elif status == 6:  # ABORTED
            current_node_id = self._get_current_node_id()

            if not self.task:
                logger.error("Task is None - cannot retry")
                return

            logger.warning(f"Goal was aborted. Target: {self.task.target_node_id}")

            # Retry logic
            if self.retry_count < self.max_retries:
                # Wait before retry
                time.sleep(2.0)
                self.retry_count += 1
                logger.info(f"Retrying navigation... Attempt {self.retry_count}/{self.max_retries}")

                # Small delay before retry
                self.node.create_timer(
                    1.0,
                    lambda: self.send_goal(self.task) if self.task else None,
                    clock=self.node.get_clock()
                )
            else:
                logger.error(f"Navigation failed after {self.max_retries} retries")
                self.current_status = RobotStatusEnum.ERROR
                self.publish_status(current_node_id, self.task.target_node_id)

                # Reset retry counter for next task
                self.retry_count = 0
                self.last_waypoint = None
                self.last_waypoint_index = 0

    def _get_current_node_id(self) -> int:
        """Get current node ID from last waypoint"""
        if self.last_waypoint is not None and self.last_waypoint < len(self.waypoints_list):
            return self.waypoints_list[self.last_waypoint].node_id
        return -1

    def feedback_callback(self, feedback_msg: NavigateThroughPoses_FeedbackMessage):
        """Handle feedback from the action server during navigation."""
        feedback: NavigateThroughPoses.Feedback = feedback_msg.feedback
        current_pose: Pose = feedback.current_pose.pose
        index = self.get_closest_waypoint_index(current_pose)

        if index != -1 and self.last_waypoint != index:
            logger.info(f"Waypoint changed: {self.last_waypoint} → {index}")
            self.last_waypoint = index
            self.current_status = RobotStatusEnum.MOVING

        current_node_id = self._get_current_node_id()
        target_node_id = self.waypoints_list[-1].node_id if self.waypoints_list else -1

        status = WPFStatus(
            status=int(self.current_status.value),
            task=self.sub_task.description if self.sub_task else "No Task",
            current_node_id=current_node_id,
            target_node_id=target_node_id,
        )

        status_json = json.dumps(asdict(status))
        msg = String()
        msg.data = status_json
        self.publisher.publish(msg)

    def publish_status(self, current_node_id: int, target_node_id: int) -> None:
        """
        Publish the current status as a JSON-formatted ROS2 String message.

        Args:
            current_node_id: The identifier of the current node.
            target_node_id: The identifier of the target node.
        """
        wpf_status = WPFStatus(
            status=int(self.current_status.value),
            task=self.sub_task.description if self.sub_task else "No Task",
            current_node_id=current_node_id,
            target_node_id=target_node_id
        )

        logger.info(f"Publishing final status: {wpf_status}")

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
        which uses its specified theta orientation.

        Args:
            waypoint_list: List of waypoints with node_id, x, y, and theta attributes.

        Returns:
            List of PoseStamped objects with positions and orientations set for navigation.
        """
        waypoint_count = len(waypoint_list)
        poses: list[PoseStamped] = []

        for index, waypoint in enumerate(waypoint_list):
            current_wp = WayPoint(**waypoint) if isinstance(waypoint, dict) else waypoint

            # Convert theta to quaternion
            w, x, y, z = euler.euler2quat(0.0, 0.0, current_wp.theta, 'sxyz')

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.node.get_clock().now().to_msg()

            if index == 0:
                # First waypoint: use current robot position
                pose.pose.position.x = float(self.robot_status.topo_map_position.x)
                pose.pose.position.y = float(self.robot_status.topo_map_position.y)
                pose.pose.position.z = 0.0

                # Use waypoint orientation
                pose.pose.orientation.x = float(x)
                pose.pose.orientation.y = float(y)
                pose.pose.orientation.z = float(z)
                pose.pose.orientation.w = float(w)

                logger.info(
                    f"Start pose: ({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}) "
                    f"theta: {current_wp.theta:.3f}")
            else:
                # Subsequent waypoints
                pose.pose.position.x = float(current_wp.x)
                pose.pose.position.y = float(current_wp.y)
                pose.pose.position.z = 0.0

                # Calculate orientation towards next waypoint or use specified theta
                next_wp = waypoint_list[index + 1] if index + 1 < waypoint_count else None

                if next_wp:
                    # Point towards next waypoint
                    dx = next_wp.x - current_wp.x
                    dy = next_wp.y - current_wp.y

                    if abs(dx) > 0.01 or abs(dy) > 0.01:  # Only calculate if distance is significant
                        angle = math.atan2(dy, dx)
                        logger.info(f"Waypoint {current_wp.node_id} Pose: {current_wp.x, current_wp.y} and theta: {angle:.3f}")
                        pose.pose.orientation.z = math.sin(angle / 2)
                        pose.pose.orientation.w = math.cos(angle / 2)
                        pose.pose.orientation.x = 0.0
                        pose.pose.orientation.y = 0.0
                    else:
                        # Waypoints too close, use specified orientation
                        pose.pose.orientation.x = float(x)
                        pose.pose.orientation.y = float(y)
                        pose.pose.orientation.z = float(z)
                        pose.pose.orientation.w = float(w)
                else:
                    # Last waypoint: use specified theta orientation
                    pose.pose.orientation.x = float(x)
                    pose.pose.orientation.y = float(y)
                    pose.pose.orientation.z = float(z)
                    pose.pose.orientation.w = float(w)
                    logger.info(f"Final waypoint {current_wp.node_id} Pose: {current_wp.x, current_wp.y} and theta: {current_wp.theta:.3f}")

            poses.append(pose)

        return poses

    def get_closest_waypoint_index(self, current_pose: Pose) -> int:
        """
        Finds the closest waypoint based on distance with hysteresis to prevent oscillation.

        Args:
            current_pose: Current robot pose

        Returns:
            Index of the closest waypoint
        """
        if not self.waypoints_list:
            return -1

        min_dist = float('inf')
        closest_index = self.last_waypoint_index
        threshold = 0.05 ** 2  # 25cm² - consider "reached" threshold
        switch_buffer = 0.3  # Must be 0.3m closer to switch to next waypoint

        # Calculate distance to current tracked waypoint
        current_wp = self.waypoints_list[self.last_waypoint_index]
        current_dist = (
            (current_wp.x - current_pose.position.x) ** 2 +
            (current_wp.y - current_pose.position.y) ** 2
        )

        # Check all waypoints
        for i, waypoint in enumerate(self.waypoints_list):
            dist_squared = (
                (waypoint.x - current_pose.position.x) ** 2 +
                (waypoint.y - current_pose.position.y) ** 2
            )

            # If within threshold, consider this waypoint reached
            if dist_squared <= threshold:
                if i != self.last_waypoint_index:
                    logger.debug(f"Reached waypoint {waypoint.node_id} (dist: {dist_squared**0.5:.3f}m)")
                self.last_waypoint_index = i
                return i

            # Only switch if significantly closer than current waypoint (hysteresis)
            if dist_squared < min_dist and dist_squared < (current_dist - switch_buffer ** 2):
                min_dist = dist_squared
                closest_index = i

        # Only update if we found a closer waypoint
        if closest_index != self.last_waypoint_index:
            self.last_waypoint_index = closest_index

        return closest_index
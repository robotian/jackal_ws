"""
Navigation Action Client using NavigateThroughPoses for robot navigation.

This module provides an action client that interfaces with Nav2's NavigateThroughPoses
action server to navigate the robot through a series of waypoints and return the
navigation status without publishing robot status updates.
"""

import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.impl.rcutils_logger import RcutilsLogger
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped

from status_interfaces.msg import WayPoint
from jackel_ops.enum import RobotStatusEnum, SubTaskEnum, NavigationStatus
from jackel_ops.dataclass import WPFStatus

class NavigationActionClient:
    """Action client for robot navigation using NavigateThroughPoses."""

    def __init__(self, node: Node):
        """
        Initialize the Navigation Action Client.

        Args:
            node_name: Name of the ROS2 node
        """
        self.node = node
        self.namespace = self.node.get_namespace().rstrip('/')
        self.logger = RcutilsLogger(self.__class__.__name__)

        # Action client for NavigateThroughPoses
        self._action_client = ActionClient(
            self.node,
            NavigateThroughPoses,
            f'{self.namespace}/navigate_through_poses'
        )

        # Internal state tracking
        self._current_status = NavigationStatus.IDLE
        self._goal_handle = None
        self._result_future = None
        self._feedback_data = None
        self._current_node_id = 0
        self._target_node_id = 0
        self._waypoints: list[WayPoint] = []

        self.logger.info(f"Navigation Action Client initialized: {self._action_client._action_name}")

    def wait_for_server(self, timeout_sec: float = 10.0) -> bool:
        """
        Wait for the action server to be available.

        Args:
            timeout_sec: Maximum time to wait for server

        Returns:
            True if server is available, False otherwise
        """
        self.logger.info("Waiting for NavigateThroughPoses action server...")
        server_available = self._action_client.wait_for_server(timeout_sec=timeout_sec)

        if server_available:
            self.logger.info("NavigateThroughPoses action server is available")
        else:
            self.logger.error("NavigateThroughPoses action server not available")

        return server_available

    def send_navigation_goal(
        self,
        waypoints: list[WayPoint],
        current_node_id: int,
        target_node_id: int,
        frame_id: str = "map"
    ) -> bool:
        """
        Send navigation goal with waypoints to the action server.

        Args:
            waypoints: List of WayPoint objects to navigate through
            current_node_id: Starting node ID
            target_node_id: Destination node ID
            frame_id: Reference frame for poses (default: "map")

        Returns:
            True if goal was sent successfully, False otherwise
        """
        if not self._action_client.server_is_ready():
            self.logger.error("Action server not ready")
            return False

        if not waypoints or len(waypoints) == 0:
            self.logger.error("No waypoints provided")
            return False

        # Store waypoint information
        self._waypoints = waypoints
        self._current_node_id = current_node_id
        self._target_node_id = target_node_id

        # Create goal message
        goal_msg = NavigateThroughPoses.Goal()

        # Convert waypoints to PoseStamped messages
        for wp in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = self.node.get_clock().now().to_msg()
            pose.pose.position.x = wp.x
            pose.pose.position.y = wp.y
            pose.pose.position.z = 0.0

            # Use theta from waypoint if available, otherwise default orientation
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0

            goal_msg.poses.append(pose)

        self.logger.info(
            f"Sending navigation goal: {len(waypoints)} waypoints from node "
            f"{current_node_id} to node {target_node_id}"
        )

        # Send goal
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )

        send_goal_future.add_done_callback(self._goal_response_callback)
        self._current_status = NavigationStatus.GOAL_SENT

        return True

    def _goal_response_callback(self, future):
        """
        Callback for goal acceptance response.

        Args:
            future: Future object containing goal handle
        """
        self._goal_handle = future.result()

        if not self._goal_handle.accepted:
            self.logger.error("Navigation goal rejected by action server")
            self._current_status = NavigationStatus.FAILED
            return

        self.logger.info("Navigation goal accepted by action server")
        self._current_status = NavigationStatus.NAVIGATING

        # Get result
        self._result_future = self._goal_handle.get_result_async()
        self._result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        """
        Callback for navigation feedback.

        Args:
            feedback_msg: Feedback message from action server
        """
        feedback = feedback_msg.feedback
        self._feedback_data = feedback

        # Log navigation progress
        current_pose = feedback.current_pose
        self.logger.debug(
            f"Navigation feedback - Position: ({current_pose.pose.position.x:.2f}, "
            f"{current_pose.pose.position.y:.2f}), "
            f"Poses remaining: {feedback.number_of_poses_remaining}"
        )

    def _result_callback(self, future):
        """
        Callback for navigation result.

        Args:
            future: Future object containing navigation result
        """
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.logger.info(
                f"Navigation succeeded! Reached node {self._target_node_id}"
            )
            self._current_status = NavigationStatus.SUCCEEDED

        elif status == GoalStatus.STATUS_ABORTED:
            self.logger.error("Navigation aborted by action server")
            self._current_status = NavigationStatus.ABORTED

        elif status == GoalStatus.STATUS_CANCELED:
            self.logger.warn("Navigation was cancelled")
            self._current_status = NavigationStatus.CANCELLED

        else:
            self.logger.error(f"Navigation failed with status: {status}")
            self._current_status = NavigationStatus.FAILED

    def cancel_navigation(self) -> bool:
        """
        Cancel the current navigation goal.

        Returns:
            True if cancellation request was sent, False otherwise
        """
        if self._goal_handle is None:
            self.logger.warn("No active goal to cancel")
            return False

        self.logger.info("Cancelling navigation goal...")
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._cancel_callback)
        return True

    def _cancel_callback(self, future):
        """
        Callback for goal cancellation.

        Args:
            future: Future object containing cancellation result
        """
        cancel_response = future.result()

        if len(cancel_response.goals_canceling) > 0:
            self.logger.info("Navigation goal successfully cancelled")
            self._current_status = NavigationStatus.CANCELLED
        else:
            self.logger.error("Failed to cancel navigation goal")

    def get_navigation_status(self) -> WPFStatus:
        """
        Get the current navigation status.

        Returns:
            WPFStatus object containing current navigation status
        """
        # Map NavigationStatus to RobotStatusEnum
        status_map = {
            NavigationStatus.IDLE: RobotStatusEnum.IDLE.value,
            NavigationStatus.GOAL_SENT: RobotStatusEnum.START_MOVING.value,
            NavigationStatus.NAVIGATING: RobotStatusEnum.MOVING.value,
            NavigationStatus.SUCCEEDED: RobotStatusEnum.DESTINATION_REACHED.value,
            NavigationStatus.FAILED: RobotStatusEnum.ERROR.value,
            NavigationStatus.CANCELLED: RobotStatusEnum.PAUSED.value,
            NavigationStatus.ABORTED: RobotStatusEnum.ERROR.value,
        }

        robot_status = status_map.get(
            self._current_status,
            RobotStatusEnum.IDLE.value
        )

        return WPFStatus(
            status=robot_status,
            task=SubTaskEnum.MOVING.name,
            current_node_id=self._current_node_id,
            target_node_id=self._target_node_id
        )

    def get_current_status(self) -> NavigationStatus:
        """
        Get the current navigation status enum.

        Returns:
            NavigationStatus enum value
        """
        return self._current_status

    def is_navigation_complete(self) -> bool:
        """
        Check if navigation is complete (succeeded, failed, or cancelled).

        Returns:
            True if navigation is complete, False otherwise
        """
        return self._current_status in [
            NavigationStatus.SUCCEEDED,
            NavigationStatus.FAILED,
            NavigationStatus.CANCELLED,
            NavigationStatus.ABORTED
        ]

    def is_navigation_active(self) -> bool:
        """
        Check if navigation is currently active.

        Returns:
            True if navigation is active, False otherwise
        """
        return self._current_status in [
            NavigationStatus.GOAL_SENT,
            NavigationStatus.NAVIGATING
        ]

    def reset(self):
        """Reset the navigation client to idle state."""
        self._current_status = NavigationStatus.IDLE
        self._goal_handle = None
        self._result_future = None
        self._feedback_data = None
        self._waypoints = []
        self.logger.info("Navigation client reset to idle state")


from dataclasses import asdict
import json
import os
from rclpy.node import Node
from rclpy.action import ActionClient, client
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.task import Future
from action_msgs.msg import GoalStatus
from std_msgs.msg import String

from opennav_docking_msgs.action._undock_robot import UndockRobot_FeedbackMessage
from opennav_docking_msgs.action import UndockRobot
from status_interfaces.msg import SubTask, UndockGoal

from jackel_ops.dataclass import DockingFeedback
from jackel_ops.enum import RobotStatusEnum

logger = RcutilsLogger(os.path.basename(__file__))


class UndockingActionClient:
    def __init__(self, node: Node):
        # super().__init__('april_tag_undocking')
        logger.info("AprilTag Undocking Node has been started.")
        self.node = node
        self.namespace = self.node.get_namespace().rstrip('/')

        self._undocking_data: UndockGoal
        self._sub_task: SubTask
        self._goal_handle: client.ClientGoalHandle
        self._current_status = RobotStatusEnum.START_DOCKING

        self._feedback_state: int = 0
        self._docking_time = 0

        self.status_map = {
            GoalStatus.STATUS_ACCEPTED: RobotStatusEnum.UNDOCKING,
            GoalStatus.STATUS_EXECUTING: RobotStatusEnum.UNDOCKING,
            GoalStatus.STATUS_SUCCEEDED: RobotStatusEnum.DONE_UNDOCKING,
            GoalStatus.STATUS_ABORTED: RobotStatusEnum.ERROR,
            GoalStatus.STATUS_CANCELING: RobotStatusEnum.IDLE,
            GoalStatus.STATUS_CANCELED: RobotStatusEnum.IDLE,
        }

        self.publisher = self.node.create_publisher(
            String, f'{self.namespace}/status/robot/undocking', 10)

        self.client = ActionClient(
            self.node, UndockRobot, f'{self.namespace}/undock_robot')

        logger.info(
            f"Undocking action client has been started. Client ID: {self.client._action_name}")

    def send_undocking_goal(self, task: SubTask | None):
        if not task:
            logger.warning(
                f"Warning while sending Goal: {task}")
            return

        self._sub_task = task

        if isinstance(self._sub_task.data, UndockGoal):
            try:
                # undocking_json = json.loads(self._sub_task.data)
                self._undocking_data = self._sub_task.data
                logger.info(f"Undocking Goal Data: {self._sub_task.data}")
            except Exception as e:
                logger.error(f"Failed to parse UndockingGoal: {e}")
                return

        self.undocking_robot()

    def undocking_robot(self):
        if not self._undocking_data:
            logger.error("Undocking data is missing. Failed to send goal.")
            return

        undock_goal_msg = UndockRobot.Goal()
        undock_goal_msg.dock_type = self._undocking_data.dock_type
        undock_goal_msg.max_undocking_time = self._undocking_data.max_undocking_time

        logger.info("Sending UndockRobot action goal.")

        if not self.client.wait_for_server(timeout_sec=5.0):
            logger.info("Undocking action server not available!")
            return

        # Send goal
        future = self.client.send_goal_async(undock_goal_msg)
        future.add_done_callback(self.done_callback)

    def done_callback(self, future: Future):
        result = future.result()
        if result is None:
            logger.error("Undocking goal result is None.")
            return

        self._goal_handle = result

        if not self._goal_handle.accepted:
            logger.warning("Undocking Goal was rejected.")
            self._active_goal = False
            return

        logger.info("Undocking Goal was accepted.")
        future_result = self._goal_handle.get_result_async()
        future_result.add_done_callback(self.result_callback)

    def result_callback(self, future: Future):
        # DockRobot_GetResult_Response
        response = future.result()
        if response is None:
            logger.error("Undocking result callback received None.")
            return

        status = response.status
        result: UndockRobot.Result = response.result

        logger.info(f"Undocking Result: {result}, Status: {status}")

        # Process the result
        status_str: str = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED"
        }.get(status, "UNKNOWN")

        self._current_status = self.status_map.get(
            status, self._current_status)

        status = DockingFeedback(
            status=self._current_status.value,
            task=self._sub_task.description,
            docking_location=self._undocking_data.dock_type,
            feedback_message=status_str,
            docking_time=self._docking_time,
            num_retries=-1
        )

        undocking_msg = String()
        undocking_msg.data = json.dumps(asdict(status))
        self.publisher.publish(undocking_msg)

        logger.info(f"Publishing Undock status message: {status}")

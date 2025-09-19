from dataclasses import asdict
import json
import os
from rclpy.action import ActionClient, client
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node
from rclpy.task import Future
from action_msgs.msg import GoalStatus
from std_msgs.msg import String

from opennav_docking_msgs.action._dock_robot import DockRobot_FeedbackMessage
from opennav_docking_msgs.action import DockRobot
from status_interfaces.msg import SubTask, DockGoal

from jackel_ops.dataclass import DockingFeedback
from jackel_ops.enum import RobotStatusEnum

logger = RcutilsLogger(os.path.basename(__file__))


class DockingActionClient:
    def __init__(self, node: Node):
        # super().__init__('april_tag_docking')
        logger.info("AprilTag Docking Node has been started.")
        self.node = node
        self.namespace = self.node.get_namespace().rstrip('/')

        self.docking_data: DockGoal
        self.sub_task: SubTask
        self.goal_handle: client.ClientGoalHandle
        self.current_status = RobotStatusEnum.START_DOCKING

        self.feedback_state: int = 0
        self.docking_time = 0

        self.status_map = {
            GoalStatus.STATUS_ACCEPTED: RobotStatusEnum.DOCKING,
            GoalStatus.STATUS_EXECUTING: RobotStatusEnum.DOCKING,
            GoalStatus.STATUS_SUCCEEDED: RobotStatusEnum.DONE_DOCKING,
            GoalStatus.STATUS_ABORTED: RobotStatusEnum.ERROR,
            GoalStatus.STATUS_CANCELING: RobotStatusEnum.IDLE,
            GoalStatus.STATUS_CANCELED: RobotStatusEnum.IDLE,
        }

        self.publisher = self.node.create_publisher(
            String, f'{self.namespace}/status/robot/docking', 10)

        self.client = ActionClient(
            self.node, DockRobot, f'{self.namespace}/dock_robot')

        logger.info(
            f"Docking action client has been started. Client ID: {self.client._action_name}")

    def send_docking_goal(self, task: SubTask | None):
        if not task:
            logger.warning(
                f"Warning while sending Goal: {task}")
            return

        self.sub_task = task

        if isinstance(self.sub_task.data, DockGoal):
            try:
                # docking_json = json.loads(self.sub_task.data)
                self.docking_data = self.sub_task.dock_goal
                logger.warning(f"Current docking Goal: {self.sub_task.dock_goal}")
            except Exception as e:
                logger.error(f"Failed to parse DockingGoal: {e}")
                return

        self.docking_robot()

    def docking_robot(self):
        if not self.docking_data:
            logger.error("Docking data not found. Failed to send goal.")
            return

        goal_msg = DockRobot.Goal()
        goal_msg.dock_id = self.docking_data.dock_id
        goal_msg.navigate_to_staging_pose = self.docking_data.navigate_to_staging_pose

        logger.info("Sending DockRobot action goal.")

        # Wait on server
        if not self.client.wait_for_server(timeout_sec=5.0):
            logger.warning("Docking action server not available!")
            return

        # Send goal
        future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        # This is the callback for when the goal is done.
        future.add_done_callback(self.done_callback)

    def cancel_docking(self, future: Future):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()

    # Receive feedback from the action server.
    # Till the robot is docked, the feedback will be published.
    def feedback_callback(self, feedback_msg: DockRobot_FeedbackMessage):
        feedback: DockRobot.Feedback = feedback_msg.feedback

        # logger.info(f"Feedback received: {feedback}")

        feedback_state = feedback.state
        feedback_state_str: str = ""

        feedback_state_str = {
            GoalStatus.STATUS_ACCEPTED: "ACCEPTED.",
            GoalStatus.STATUS_EXECUTING: "EXECUTING.",
            GoalStatus.STATUS_CANCELING: "CANCELING.",
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED.",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED"
        }.get(feedback_state, "UNKNOWN")

        # publish the docking status till the robot is docked

        if self.feedback_state != feedback_state:
            self.feedback_state = feedback_state
            logger.info(
                f"Feedback received: {feedback.state} | "
                f"Docking Action: {feedback_state_str} | "
                f"Docking time: {feedback.docking_time.sec} seconds | "
                f"Number of Retry: {feedback.num_retries}")

            self.docking_time = feedback.docking_time.sec

            self.current_status = self.status_map.get(
                feedback_state, self.current_status)

        status = DockingFeedback(
            status=self.current_status.value,
            task=self.sub_task.description,
            docking_location=self.docking_data.dock_id,
            feedback_message=feedback_state_str,
            docking_time=self.docking_time,
            num_retries=feedback.num_retries
        )

        docking_msg = String()
        docking_msg.data = json.dumps(asdict(status))
        self.publisher.publish(docking_msg)

    def done_callback(self, future: Future):
        result = future.result()

        if result is None:
            logger.error(
                "Result is None for Docking. Action server may not have completed successfully.")
            return

        self.goal_handle = result

        if not self.goal_handle.accepted:
            logger.info('Goal rejected :(')
            return

        logger.info('Goal accepted :)')
        future_result = self.goal_handle.get_result_async()
        future_result.add_done_callback(self.result_callback)

    def result_callback(self, future: Future):
        # DockRobot_GetResult_Response
        response = future.result()
        if response is not None:
            status_str: str
            status = response.status
            result: DockRobot.Result = response.result

            logger.info(f"Result: {result}")

            # Process the result
            status_str = {
                GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                GoalStatus.STATUS_ABORTED: "ABORTED",
                GoalStatus.STATUS_CANCELED: "CANCELED"
            }.get(status, "UNKNOWN")

            self.current_status = self.status_map.get(
                status, self.current_status)

            logger.info(
                f"Result: Success: {result.success} | Error Code: {result.error_code} | "
                f"Message: {status_str} | Number of Retry: {result.num_retries}")

            status = DockingFeedback(
                status=self.current_status.value,
                task=self.sub_task.description,
                docking_location=self.docking_data.dock_id,
                feedback_message=status_str,
                docking_time=self.docking_time,
                num_retries=result.num_retries
            )

            logger.info(
                f"Result message: {status}")

            docking_msg = String()
            docking_msg.data = json.dumps(asdict(status))
            self.publisher.publish(docking_msg)
        else:
            logger.info(
                "Result is None. Action server may not have completed successfully.")
            return

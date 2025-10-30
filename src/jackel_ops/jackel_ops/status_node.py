import json
import pyproj
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import BatteryState, NavSatFix, Imu
from std_msgs.msg import Bool, String
from status_interfaces.msg import RobotStatus, Task, SubTask, UndockGoal

from jackel_ops.april_tag_docking import DockingActionClient
from jackel_ops.april_tag_undocking import UndockingActionClient
from jackel_ops.navigation_action_client import NavigationActionClientUsingNTP
from jackel_ops.dataclass import DockingFeedback, WPFStatus
from jackel_ops.enum import OnlineFlagEnum, RobotStatusEnum


class StatusNode(Node):
    def __init__(self):
        super().__init__('status_node')
        self.get_logger().info('StatusNode has been started.')

        # Setting namespace for Node
        self.namespace = self.get_namespace().rstrip('/')

        # Startup state management
        self.is_initialized = False
        self.is_at_docking_station = False
        self.startup_undock_complete = False
        self.checking_initial_position = False

        # Current state
        self.current_status = RobotStatusEnum.IDLE
        self.previous_status = RobotStatusEnum.IDLE
        self.current_task: Task | None = None
        self.current_sub_task: SubTask | None = None
        self.current_sub_task_index = 0
        self.current_node_id = 0
        self.current_load_status = 0.0
        self.last_handled_task_id: int | None = None
        self.last_handled_subtask_type: int | None = None
        self.status_transition_pending = False

        # Subscriptions messages
        self.battery_status: BatteryState = BatteryState()
        self.task: Task = Task()
        self.gps_status: NavSatFix = NavSatFix()
        self.pose_status: PoseWithCovarianceStamped = PoseWithCovarianceStamped()
        self.imu_status: Imu = Imu()
        self.estop_status: Bool = Bool()
        self.wpf_status: String = String()
        self.docking_status: String = String()
        self.undocking_status: String = String()

        # Initialize subscriptions
        self._init_subscriptions()

        # Dynamic subscriptions (created/destroyed as needed)
        self.wpf_sub = None
        self.docking_sub = None
        self.undocking_sub = None

        # Publishers
        self.robot_state_pub = self.create_publisher(
            RobotStatus, f'{self.namespace}/status/robot', 10)

        self.timer = self.create_timer(1.0, self.timer_callback)

        # Action Clients
        self.navigation = NavigationActionClientUsingNTP(self)
        self.april_tag_docking = DockingActionClient(self)
        self.april_tag_undocking = UndockingActionClient(self)

        # Schedule initial position check after 2 seconds
        self.create_timer(2.0, self.check_initial_position, clock=self.get_clock())

    def _init_subscriptions(self):
        """Initialize all static subscriptions"""
        self.battery_sub = self.create_subscription(
            BatteryState,
            f'{self.namespace}/platform/bms/state',
            lambda msg: setattr(self, 'battery_status', msg),
            qos_profile_sensor_data)

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            f"{self.namespace}/rigidbody_1/pose",
            lambda msg: setattr(self, 'pose_status', msg),
            10)

        self.imu_sub = self.create_subscription(
            Imu,
            f'{self.namespace}/sensors/imu_0/magnetic_field',
            lambda msg: setattr(self, 'imu_status', msg),
            qos_profile_sensor_data)

        self.estop_sub = self.create_subscription(
            Bool,
            f'{self.namespace}/platform/emergency_stop',
            lambda msg: setattr(self, 'estop_status', msg),
            qos_profile_sensor_data)

        self.task_sub = self.create_subscription(
            Task,
            f'{self.namespace}/status/task',
            self._task_callback,
            10)

    def _task_callback(self, msg: Task):
        """Dedicated task callback with validation"""
        is_new_task = msg.task_id != self.last_handled_task_id

        # Check if subtask changed
        is_new_subtask = False
        if isinstance(msg.sub_tasks, list) and len(msg.sub_tasks) > 0:
            first_subtask = msg.sub_tasks[0]
            if isinstance(first_subtask, SubTask):
                is_new_subtask = first_subtask.type != self.last_handled_subtask_type

        if is_new_task:
            self.get_logger().info(
                f"Received NEW TASK: {msg.description} (ID: {msg.task_id})")
            self.current_sub_task_index = 0
            self.last_handled_subtask_type = None
            self.status_transition_pending = False
        elif is_new_subtask:
            self.get_logger().info(
                f"Received NEW SUBTASK for task ID: {msg.task_id}")
            self.current_sub_task_index = 0
            self.status_transition_pending = False
            if self.current_status == RobotStatusEnum.JOB_DONE:
                self._transition_status(RobotStatusEnum.IDLE)

        self.task = msg

    def timer_callback(self):
        """Main timer callback - publishes status"""
        robot_status = RobotStatus()
        robot_status.header.stamp = self.get_clock().now().to_msg()
        robot_status.robot_namespace = self.namespace.replace(r'/', '')

        # Update sensor statuses
        self.set_battery_status(robot_status)
        self.set_estop_status(robot_status)
        self.set_location_status(robot_status)

        # Handle error recovery first
        if self.current_status in [RobotStatusEnum.ERROR, RobotStatusEnum.ABNORMAL]:
            self.handle_error_recovery(robot_status)
        # Handle startup undocking or normal task processing
        elif not self.startup_undock_complete:
            self.startup_undocking(robot_status)
        else:
            self.handle_task(robot_status)

        # Publish final status
        robot_status.status = self.current_status.value
        robot_status.current_node_id = self.current_node_id
        robot_status.load_status = self.current_load_status
        self.robot_state_pub.publish(robot_status)

    def _cleanup_subscriptions(self):
        """Clean up all dynamic subscriptions"""
        if self.wpf_sub is not None:
            self.destroy_subscription(self.wpf_sub)
            self.wpf_sub = None
        if self.docking_sub is not None:
            self.destroy_subscription(self.docking_sub)
            self.docking_sub = None
        if self.undocking_sub is not None:
            self.destroy_subscription(self.undocking_sub)
            self.undocking_sub = None

    # ==================== STARTUP INITIALIZATION ====================

    def check_initial_position(self):
        """Check initial robot position after startup"""
        docking_threshold = 0.2

        if self.is_initialized:
            return

        self.checking_initial_position = True

        if not self.pose_status or not self.pose_status.pose:
            self.get_logger().warning("No pose data available - retrying in 1s")
            self.create_timer(1.0, self.check_initial_position, clock=self.get_clock())
            return

        current_pos = self.pose_status.pose.pose.position
        distance = self._calculate_distance_to_dock(current_pos)

        self.get_logger().info(
            f"Initial position: ({current_pos.x:.3f}, {current_pos.y:.3f}), "
            f"Distance to dock: {distance:.3f}m")

        if distance <= docking_threshold:
            self.is_at_docking_station = True
            self.get_logger().info("Robot at docking station - will undock before tasks")
        else:
            self.is_at_docking_station = False
            self.startup_undock_complete = True
            self.get_logger().info("Robot not at docking station - ready for tasks")

        self.is_initialized = True
        self.checking_initial_position = False

    def _calculate_distance_to_dock(self, current_position) -> float:
        """Calculate Euclidean distance to docking station"""
        dock_x = 1.15
        dock_y = 1.93

        dx = current_position.x - dock_x
        dy = current_position.y - dock_y

        return (dx**2 + dy**2) ** 0.5

    def startup_undocking(self, robot_status: RobotStatus):
        """Handle undocking at startup if at docking station"""
        if not self.is_initialized:
            robot_status.task = "Initializing - checking position..."
            return

        if not self.is_at_docking_station:
            self.startup_undock_complete = True
            return

        # Process undocking feedback
        if self.undocking_sub is not None and self.undocking_status.data:
            self._handle_startup_undocking_status(robot_status)
            return

        # State machine for startup undocking
        if self.current_status == RobotStatusEnum.IDLE:
            self.get_logger().info("Starting startup undocking...")
            self._transition_status(RobotStatusEnum.START_UNDOCKING)
            robot_status.task = "Startup: Preparing to undock"

        elif self.current_status == RobotStatusEnum.START_UNDOCKING:
            self._transition_status(RobotStatusEnum.UNDOCKING)
            robot_status.task = "Startup: Undocking from station"

            if self.undocking_sub is None:
                self.undocking_sub = self.create_subscription(
                    String,
                    f'{self.namespace}/status/robot/undocking',
                    lambda msg: setattr(self, 'undocking_status', msg),
                    10)

                startup_subtask = SubTask()
                startup_subtask.type = SubTask.UNDOCKING if hasattr(SubTask, 'UNDOCKING') else 0
                startup_subtask.description = "Startup Undocking"
                startup_subtask.undock_goal = UndockGoal(
                    dock_type='jackal_dock',
                    max_undocking_time=30.0
                    )

                self.april_tag_undocking.send_undocking_goal(startup_subtask)

        elif self.current_status == RobotStatusEnum.UNDOCKING:
            robot_status.task = "Startup: Undocking in progress..."

        elif self.current_status == RobotStatusEnum.DONE_UNDOCKING:
            self.get_logger().info("Startup undocking complete")
            self.startup_undock_complete = True
            self._transition_status(RobotStatusEnum.IDLE)
            robot_status.task = "Startup undocking complete"

    def _handle_startup_undocking_status(self, robot_status: RobotStatus):
        """Handle undocking status during startup"""
        if not self.undocking_status.data:
            return

        try:
            data_dict = json.loads(self.undocking_status.data)
            status_data = DockingFeedback(**data_dict)

            robot_status.task = f"Startup: {status_data.task}"

            status_map = {
                RobotStatusEnum.UNDOCKING.value: RobotStatusEnum.UNDOCKING,
                RobotStatusEnum.DONE_UNDOCKING.value: RobotStatusEnum.DONE_UNDOCKING,
                RobotStatusEnum.ERROR.value: RobotStatusEnum.ERROR,
            }

            new_status = status_map.get(status_data.status)
            if new_status and new_status != self.current_status:
                self._transition_status(new_status)

            if status_data.status in (RobotStatusEnum.DONE_UNDOCKING.value,
                                     RobotStatusEnum.ERROR.value):
                if self.undocking_sub is not None:
                    self.destroy_subscription(self.undocking_sub)
                    self.undocking_sub = None
                # Clear status message
                self.undocking_status.data = ''

                if status_data.status == RobotStatusEnum.ERROR.value:
                    self.get_logger().error("Startup undocking failed!")
                    self.startup_undock_complete = True
                    self._transition_status(RobotStatusEnum.ERROR)

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse startup undocking status: {e}")

    # ==================== TASK HANDLING ====================

    def _transition_status(self, new_status: RobotStatusEnum):
        """Safe status transition with logging"""
        if self.current_status != new_status:
            self.previous_status = self.current_status
            self.current_status = new_status
            self.get_logger().info(
                f"Status transition: {self.previous_status.name} -> {self.current_status.name}")

    def handle_task(self, robot_status: RobotStatus):
        """Main task handling logic"""
        if not self.task or self.task.description == "" or self.task.job_schedule == "":
            if self.current_status != RobotStatusEnum.IDLE:
                self._transition_status(RobotStatusEnum.IDLE)
                self._cleanup_subscriptions()
            return

        self.current_task = self.task

        # self.get_logger().info(f"Current Task: {self.current_task.description}")

        # CRITICAL: Check for low battery FIRST
        if self._check_low_battery():
            return

        # Process action client status updates
        self._process_action_status(robot_status)

        # Update current subtask
        self._update_current_subtask()

        # Update robot status message
        robot_status.crop_type = self.current_task.crop_type
        robot_status.target_node_id = self.current_task.target_node_id

        if self.current_sub_task is not None:
            robot_status.task = self.current_sub_task.description
        else:
            robot_status.task = self.current_task.description

        # State machine: Handle current status
        if self.current_status in (RobotStatusEnum.IDLE, RobotStatusEnum.JOB_DONE):
            self._handle_task_start()
        else:
            self._execute_current_subtask()

    def _check_low_battery(self) -> bool:
        """Check battery and handle low battery condition"""
        if not self.task or not self.current_task:
            return False

        # Normalize battery for consistent checking
        battery_pct = self._normalize_battery_percentage(self.battery_status.percentage)

        if (self.task.task_type != Task.CHARGING_TASK and battery_pct <= 50.0):
            self.get_logger().warning(f"Battery Low! {battery_pct:.1f}%")

            if self.current_status in [RobotStatusEnum.START_MOVING, RobotStatusEnum.MOVING]:
                self.get_logger().warning("Cancelling navigation due to low battery")
                self.navigation.cancel_goal()
                self._cleanup_subscriptions()

            self._transition_status(RobotStatusEnum.ERROR)
            return True

        # Recover from low battery error if charging task received
        if (self.current_status == RobotStatusEnum.ERROR and
            self.current_task.task_type == Task.CHARGING_TASK):
            self.get_logger().info("Recovering from low battery - charging task received")
            self._cleanup_subscriptions()
            self._transition_status(RobotStatusEnum.IDLE)

        return False

    def _process_action_status(self, robot_status: RobotStatus):
        """Process status updates from action clients"""
        if self.wpf_sub is not None and self.wpf_status.data:
            self.handle_way_point_follower(robot_status)
        elif self.docking_sub is not None and self.docking_status.data:
            self.handle_docking(robot_status)
        elif self.undocking_sub is not None and self.undocking_status.data:
            self.handle_undocking(robot_status)

    def _update_current_subtask(self):
        """Update current subtask based on index"""
        if not self.current_task:
            return

        if isinstance(self.current_task.sub_tasks, list) and len(self.current_task.sub_tasks) > 0:
            if self.current_sub_task_index < len(self.current_task.sub_tasks):
                self.current_sub_task = self.current_task.sub_tasks[self.current_sub_task_index]
            else:
                self.current_sub_task = None

    def _handle_task_start(self):
        """Handle new task start"""
        if not self.current_task:
            return

        if self.current_task.task_id != self.last_handled_task_id:
            self.get_logger().info(
                f"Starting Task: {self.current_task.description} | "
                f"Task ID: {self.current_task.task_id} | "
                f"Current Node: {self.current_node_id} | "
                f"Target Node: {self.current_task.target_node_id}")

            self.last_handled_task_id = self.current_task.task_id
            self.last_handled_subtask_type = None
            self._transition_status(RobotStatusEnum.JOB_START)

        elif self.current_status == RobotStatusEnum.JOB_DONE:
            self._transition_status(RobotStatusEnum.IDLE)

    def _execute_current_subtask(self):
        """Execute the current subtask based on type"""
        if not isinstance(self.current_sub_task, SubTask):
            return

        # Check if new subtask type
        if self.current_sub_task.type != self.last_handled_subtask_type:
            self.get_logger().info(
                f"Executing subtask: {self.current_sub_task.description} "
                f"(Type: {self.current_sub_task.type})")
            self.last_handled_subtask_type = self.current_sub_task.type
            self.status_transition_pending = False

        task_handlers = {
            SubTask.MOVING: self.sub_task_moving,
            SubTask.HARVESTING: self.sub_task_harvesting,
            SubTask.DOCKING: self.sub_task_docking,
            SubTask.LOADING: self.sub_task_loading,
            SubTask.CHARGING: self.sub_task_charging,
            SubTask.UNLOADING: self.sub_task_unloading,
        }

        handler = task_handlers.get(self.current_sub_task.type)

        # self.get_logger().info(f"Calling function: {handler}")

        if handler:
            handler()
        else:
            self.get_logger().warning(f"Unknown subtask type: {self.current_sub_task.type}")

    # ==================== ACTION CLIENT HANDLERS ====================

    def handle_way_point_follower(self, robot_status: RobotStatus):
        """Handle navigation waypoint follower status"""
        if not self.wpf_status.data:
            return

        try:
            data_dict = json.loads(self.wpf_status.data)
            wpf_data = WPFStatus(**data_dict)

            robot_status.task = wpf_data.task
            robot_status.current_node_id = wpf_data.current_node_id

            status = wpf_data.status

            status_map = {
                RobotStatusEnum.MOVING.value: RobotStatusEnum.MOVING,
                RobotStatusEnum.DESTINATION_REACHED.value: RobotStatusEnum.DESTINATION_REACHED,
                RobotStatusEnum.ERROR.value: RobotStatusEnum.ERROR,
            }

            new_status = status_map.get(status)
            if new_status and new_status != self.current_status:
                self._transition_status(new_status)

                # Update node immediately on destination reached
                if new_status == RobotStatusEnum.DESTINATION_REACHED:
                    self.current_node_id = wpf_data.current_node_id
                    self.get_logger().info(f"Navigation complete - arrived at node {self.current_node_id}")

                    # Print current location
                    if self.pose_status and self.pose_status.pose:
                        pos = self.pose_status.pose.pose.position
                        orient = self.pose_status.pose.pose.orientation
                        self.get_logger().info(
                            f"Current location: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}, "
                            f"orientation: x={orient.x:.3f}, y={orient.y:.3f}, z={orient.z:.3f}, w={orient.w:.3f}")
                    else:
                        self.get_logger().warning("Pose data not available")

            # Clean up when done
            if status in (RobotStatusEnum.DESTINATION_REACHED.value,
                         RobotStatusEnum.ERROR.value):
                if self.wpf_sub is not None:
                    self.get_logger().info("Cleaning up navigation subscription")
                    self.destroy_subscription(self.wpf_sub)
                    self.wpf_sub = None
                self.status_transition_pending = False
                self.wpf_status.data = ''  # Clear message

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse navigation status: {e}")

    def handle_docking(self, robot_status: RobotStatus):
        """Handle docking status"""
        if not self.docking_status.data or not self.current_task:
            return

        try:
            data_dict = json.loads(self.docking_status.data)
            status_data = DockingFeedback(**data_dict)

            robot_status.task = status_data.task
            self.current_node_id = self.current_task.target_node_id

            status = status_data.status

            status_map = {
                RobotStatusEnum.DOCKING.value: RobotStatusEnum.DOCKING,
                RobotStatusEnum.DONE_DOCKING.value: RobotStatusEnum.DONE_DOCKING,
                RobotStatusEnum.ERROR.value: RobotStatusEnum.ERROR,
            }

            new_status = status_map.get(status)
            if new_status and new_status != self.current_status:
                self._transition_status(new_status)

            # Clean up when done
            if status == RobotStatusEnum.DONE_DOCKING.value:
                if self.docking_sub is not None:
                    self.get_logger().info("Cleaning up docking subscription")
                    self.destroy_subscription(self.docking_sub)
                    self.docking_sub = None
                self.status_transition_pending = False
                self.docking_status.data = ''  # Clear message

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse docking status: {e}")

    def handle_undocking(self, robot_status: RobotStatus):
        """Handle undocking status"""
        if not self.undocking_status.data or not self.current_task:
            return

        try:
            data_dict = json.loads(self.undocking_status.data)
            status_data = DockingFeedback(**data_dict)

            robot_status.task = status_data.task

            status_map = {
                RobotStatusEnum.UNDOCKING.value: RobotStatusEnum.UNDOCKING,
                RobotStatusEnum.DONE_UNDOCKING.value: RobotStatusEnum.DONE_UNDOCKING,
                RobotStatusEnum.ERROR.value: RobotStatusEnum.ERROR,
            }

            new_status = status_map.get(status_data.status)
            if new_status and new_status != self.current_status:
                self._transition_status(new_status)

            # Clean up and finalize
            if status_data.status in (RobotStatusEnum.DONE_UNDOCKING.value,
                                     RobotStatusEnum.ERROR.value):
                if self.undocking_sub is not None:
                    self.get_logger().info("Cleaning up undocking subscription")
                    self.destroy_subscription(self.undocking_sub)
                    self.undocking_sub = None
                self.current_node_id = self.current_task.target_node_id
                self.status_transition_pending = False
                self.undocking_status.data = ''  # Clear message

                if self.current_status == RobotStatusEnum.DONE_UNDOCKING:
                    self._transition_status(RobotStatusEnum.JOB_DONE)

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse undocking status: {e}")

    # ==================== ERROR RECOVERY =======================

    def handle_error_recovery(self, robot_status: RobotStatus):
        """
        Handle error recovery by resetting robot to nearest node.
        Called when status is ERROR (94) or ABNORMAL (99).
        """
        if self.current_status not in [RobotStatusEnum.ERROR, RobotStatusEnum.ABNORMAL]:
            return

        self.get_logger().warning(
            f"Error recovery initiated. Current status: {self.current_status.name}")

        # # Get current position
        # if self.pose_status and self.pose_status.pose:
        #     current_pos = self.pose_status.pose.pose.position
        #     self.get_logger().info(
        #         f"Current position: x={current_pos.x:.3f}, y={current_pos.y:.3f}")

        #     # Find nearest node (you'll need to implement this based on your node map)
        #     nearest_node = self._find_nearest_node(current_pos)

        #     if nearest_node is not None:
        #         self.current_node_id = nearest_node
        #         self.get_logger().info(
        #             f"Reset to nearest node: {nearest_node}")

        # Clean up any active subscriptions
        self._cleanup_subscriptions()

        # Cancel any active goals
        if self.navigation.goal_in_progress:
            self.get_logger().info("Cancelling active navigation goal")
            self.navigation.cancel_goal()

        # Reset state variables
        self.status_transition_pending = False
        self.current_sub_task = None
        self.current_sub_task_index = 0

        # Log the error for debugging
        robot_status.task = f"Error Recovery - Reset to node {self.current_node_id}"

        # Transition to IDLE after a delay
        self.get_logger().info("Transitioning to IDLE state")
        self._transition_status(RobotStatusEnum.IDLE)

    # ==================== SUB-TASK HANDLERS ====================

    def sub_task_moving(self):
        """Handle moving subtask"""
        if not self.current_task:
            return

        if self.current_status == RobotStatusEnum.JOB_START:
            self.get_logger().info("Transitioning to START_MOVING")
            self._transition_status(RobotStatusEnum.START_MOVING)

        elif self.current_status == RobotStatusEnum.START_MOVING:
            # Check if subscription already exists
            if self.wpf_sub is not None:
                self.get_logger().warning("Navigation subscription exists - skipping")
                return

            self.get_logger().info(
                f"Starting navigation: node {self.current_node_id} → {self.current_task.target_node_id}")

            self._transition_status(RobotStatusEnum.MOVING)

            # Create subscription
            self.wpf_sub = self.create_subscription(
                String,
                f'{self.namespace}/status/robot/navigation',
                lambda msg: setattr(self, 'wpf_status', msg),
                10)

            # Reset navigation state
            self.navigation.is_goal_cancelled = False
            self.navigation.retry_count = 0
            self.navigation.last_waypoint = None
            self.navigation.last_waypoint_index = 0

            # Send goal
            self.navigation.send_goal(self.current_task)
            self.status_transition_pending = True

    def sub_task_docking(self):
        """Handle docking subtask"""
        if self.current_status == RobotStatusEnum.DESTINATION_REACHED:
            self.get_logger().info("Transitioning to START_DOCKING")
            self._transition_status(RobotStatusEnum.START_DOCKING)

        elif self.current_status == RobotStatusEnum.START_DOCKING:
            if self.docking_sub is not None:
                self.get_logger().warning("Docking subscription exists - skipping")
                return

            self.get_logger().info("Starting docking sequence")
            self._transition_status(RobotStatusEnum.DOCKING)

            # self.get_logger().info(f"Current Task: {self.current_task}")
            # self.get_logger().info(f"Current Docking Goal: {self.current_sub_task}")

            self.docking_sub = self.create_subscription(
                String,
                f'{self.namespace}/status/robot/docking',
                lambda msg: setattr(self, 'docking_status', msg),
                10)
            self.april_tag_docking.send_docking_goal(self.current_sub_task)
            self.status_transition_pending = True

    def sub_task_undocking(self):
        """Handle undocking subtask"""
        if self.current_status == RobotStatusEnum.START_UNDOCKING:
            if self.undocking_sub is not None:
                self.get_logger().warning("Undocking subscription exists - skipping")
                return

            self.get_logger().info("Starting undocking sequence")
            self._transition_status(RobotStatusEnum.UNDOCKING)

            self.undocking_sub = self.create_subscription(
                String,
                f'{self.namespace}/status/robot/undocking',
                lambda msg: setattr(self, 'undocking_status', msg),
                10)
            self.april_tag_undocking.send_undocking_goal(self.current_sub_task)
            self.status_transition_pending = True

    def sub_task_charging(self):
        """Handle charging subtask"""
        if self.current_status == RobotStatusEnum.DONE_DOCKING:
            self._transition_status(RobotStatusEnum.START_CHARGING)

        elif self.current_status == RobotStatusEnum.START_CHARGING:
            self._transition_status(RobotStatusEnum.CHARGING)

        elif self.current_status == RobotStatusEnum.CHARGING:
            battery_pct = self._normalize_battery_percentage(self.battery_status.percentage)
            self.get_logger().debug(f"Battery Charging: {battery_pct:.2f}%")

            if battery_pct >= 99.0:
                self.get_logger().debug(f"Battery Charged: {battery_pct:.2f}%")
                self._transition_status(RobotStatusEnum.DONE_CHARGING)

        elif self.current_status == RobotStatusEnum.DONE_CHARGING:
            self._transition_status(RobotStatusEnum.START_UNDOCKING)
            self.sub_task_undocking()

    def sub_task_harvesting(self):
        """Handle harvesting subtask"""
        if self.current_status == RobotStatusEnum.DESTINATION_REACHED:
            self._transition_status(RobotStatusEnum.START_HARVESTING)

        elif self.current_status == RobotStatusEnum.START_HARVESTING:
            self._transition_status(RobotStatusEnum.HARVESTING)
            self.get_logger().info("Harvesting started")

        elif self.current_status == RobotStatusEnum.HARVESTING:
            # TODO: Integrate actual harvesting logic
            self.get_logger().info("Harvesting complete")
            self._transition_status(RobotStatusEnum.DONE_HARVESTING)

    def sub_task_loading(self):
        """Handle loading subtask"""
        if self.current_status == RobotStatusEnum.DONE_HARVESTING:
            self._transition_status(RobotStatusEnum.START_LOADING)

        elif self.current_status == RobotStatusEnum.START_LOADING:
            self._transition_status(RobotStatusEnum.LOADING)
            self.get_logger().info("Loading started")

        elif self.current_status == RobotStatusEnum.LOADING:
            # TODO: Integrate actual loading logic
            self.get_logger().info("Loading complete")
            self._transition_status(RobotStatusEnum.DONE_LOADING)

        elif self.current_status == RobotStatusEnum.DONE_LOADING:
            self.current_load_status = min(self.current_load_status + 20.0, 100.0)
            self.get_logger().info(f"Load status: {self.current_load_status}%")
            self._transition_status(RobotStatusEnum.JOB_DONE)

    def sub_task_unloading(self):
        """Handle unloading subtask"""
        if self.current_status == RobotStatusEnum.DONE_DOCKING:
            self._transition_status(RobotStatusEnum.START_UNLOADING)

        elif self.current_status == RobotStatusEnum.START_UNLOADING:
            self._transition_status(RobotStatusEnum.UNLOADING)
            self.get_logger().info("Unloading started")

        elif self.current_status == RobotStatusEnum.UNLOADING:
            # TODO: Integrate actual unloading logic
            self.get_logger().info("Unloading complete")
            self._transition_status(RobotStatusEnum.DONE_UNLOADING)

        elif self.current_status == RobotStatusEnum.DONE_UNLOADING:
            self.current_load_status = 0.0
            self._transition_status(RobotStatusEnum.START_UNDOCKING)
            self.get_logger().info("Unloading done, starting undocking")
            self.sub_task_undocking()

    # ==================== STATUS SETTERS ====================

    def set_battery_status(self, robot_status: RobotStatus):
        """Update battery status information"""
        robot_status.battery_level = self.battery_status.percentage

        if self.battery_status.capacity > 0.0 and self.battery_status.current > 0.0:
            battery_pct = self._normalize_battery_percentage(self.battery_status.percentage)
            time_remaining = (self.battery_status.capacity *
                            (battery_pct / 100.0) /
                            self.battery_status.current)
            robot_status.operation_hours_after_charging = self.convert_to_hhmmss(time_remaining)
        else:
            robot_status.operation_hours_after_charging = "00 hours 00 minutes remaining approx..."

    def set_estop_status(self, robot_status: RobotStatus):
        """Update emergency stop status"""
        robot_status.online_flag = (self.estop_status.data if self.estop_status.data
                                   else OnlineFlagEnum.ONLINE.value)

    def set_location_status(self, robot_status: RobotStatus):
        """Update robot location from pose data"""
        if self.pose_status and self.pose_status.pose:
            robot_status.topo_map_position = self.pose_status.pose.pose.position
            robot_status.topo_map_orientation = self.pose_status.pose.pose.orientation

    # ==================== HELPER METHODS ====================

    def _normalize_battery_percentage(self, percentage: float) -> float:
        """
        Normalize battery percentage to 0-100 range.
        Handles both 0.0-1.0 and 0-100 representations.
        """
        if percentage <= 1.0:
            return percentage * 100.0
        else:
            return percentage

    def convert_to_hhmmss(self, hours: float) -> str:
        """Convert hours to HH:MM formatted string"""
        seconds = int(hours * 3600)
        minutes, seconds = divmod(seconds, 60)
        hours, minutes = divmod(minutes, 60)
        return f"{hours:02} hours {minutes:02} minutes remaining approximately."

    def scale_value(self, value: float, original_min: float, original_max: float,
                   target_min: float, target_max: float) -> float:
        """Scale value from one range to another"""
        if original_max == original_min:
            raise ValueError("Original range cannot be zero.")

        return (((value - original_min) / (original_max - original_min)) *
                (target_max - target_min) + target_min)

    def lat_lon_to_cartesian(self, lat: float, lon: float, alt: float) -> tuple:
        """Convert lat/lon/alt to Cartesian coordinates"""
        wgs84 = pyproj.CRS("EPSG:4326")
        geocentric = pyproj.CRS("EPSG:4978")
        transformer = pyproj.Transformer.from_crs(wgs84, geocentric, always_xy=True)
        return transformer.transform(lon, lat, alt)


def main(args=None):
    rclpy.init(args=args)
    node = StatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

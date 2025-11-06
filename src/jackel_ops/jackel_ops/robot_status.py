"""
Status Node for Robot Control System

This module provides a simplified state machine for managing robot operations
including navigation, docking, undocking, charging, harvesting, loading, and unloading.
"""

import pyproj
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import BatteryState, NavSatFix, Imu
from std_msgs.msg import Bool
from status_interfaces.msg import RobotStatus, Task, SubTask, UndockGoal

from jackel_ops.april_tag_docking import DockingActionClient
from jackel_ops.april_tag_undocking import UndockingActionClient
from jackel_ops.navigation_action_client import NavigationActionClient
from jackel_ops.enum import NavigationStatus, OnlineFlagEnum, RobotStatusEnum
from jackel_ops.config_loader import ConfigLoader


# =============================================================================
# CONFIGURATION - All hardcoded values in one place
# =============================================================================
class RobotConfig:
    """Centralized configuration for robot operations"""

    # Navigation settings
    MAX_NAVIGATION_RETRIES = 3
    NAVIGATION_RETRY_DELAY = 2.0  # seconds

    # Docking settings
    MAX_DOCKING_RETRIES = 2
    DOCKING_RETRY_DELAY = 3.0  # seconds
    DOCKING_THRESHOLD = 0.2  # meters - distance to consider "at dock"

    # Undocking settings
    MAX_UNDOCKING_RETRIES = 2
    UNDOCKING_RETRY_DELAY = 3.0  # seconds
    STAGING_THRESHOLD = 0.05  # meters - staging area crossing threshold

    # Battery settings
    LOW_BATTERY_THRESHOLD = 50.0  # percent
    FULL_BATTERY_THRESHOLD = 99.0  # percent

    # Loading settings
    LOAD_INCREMENT = 20.0  # percent per load operation

    # Timing
    TIMER_PERIOD = 1.0  # seconds - main loop frequency
    INITIAL_POSITION_CHECK_DELAY = 2.0  # seconds

    # Server timeout
    ACTION_SERVER_TIMEOUT = 5.0  # seconds


# =============================================================================
# MAIN STATUS NODE
# =============================================================================
class StatusNode(Node):
    """Main node for managing robot status and task execution"""

    def __init__(self):
        super().__init__('status_node')
        self.get_logger().info('StatusNode has been started.')

        # Load dock configuration
        self._load_dock_config()

        # Initialize robot namespace
        self.namespace = self.get_namespace().rstrip('/')

        # Initialize state variables
        self._init_state_variables()

        # Initialize sensors and subscriptions
        self._init_sensor_data()
        self._init_subscriptions()

        # Initialize publishers
        self._init_publishers()

        # Initialize action clients
        self._init_action_clients()

        # Start main timer and initial position check
        self.timer = self.create_timer(RobotConfig.TIMER_PERIOD, self.timer_callback)
        self.create_timer(
            RobotConfig.INITIAL_POSITION_CHECK_DELAY,
            self.check_initial_position,
            clock=self.get_clock()
        )

    # =========================================================================
    # INITIALIZATION METHODS
    # =========================================================================

    def _load_dock_config(self):
        """Load charging station configuration"""
        config = ConfigLoader('project_bringup', 'config/j100/docking_config.yaml')
        self.charging_x, self.charging_y, self.charging_theta, self.staging_x = config.load_docking_config()
        self.get_logger().info(
            f"Charging Dock: ({self.charging_x:.2f}, {self.charging_y:.2f}, {self.charging_theta:.2f})")
        self.get_logger().info(
            f"Staging Pose: ({self.staging_x:.2f}, {self.charging_y:.2f})")

    def _init_state_variables(self):
        """Initialize all state tracking variables"""
        # Startup state
        self.is_initialized = False
        self.is_at_docking_station = False
        self.startup_undock_complete = False

        # Current robot state
        self.current_status = RobotStatusEnum.IDLE
        self.previous_status = RobotStatusEnum.IDLE

        # Task management
        self.current_task: Task | None = None
        self.current_sub_task: SubTask | None = None
        self.current_sub_task_index = 0
        self.last_handled_task_id: int | None = None
        self.last_handled_subtask_type: int | None = None

        # Robot state
        self.current_node_id = 0
        self.current_load_status = 0.0

        # Retry management
        self.navigation_retry_count = 0
        self.docking_retry_count = 0
        self.undocking_retry_count = 0

        # Job simulation tracking (for placeholders)
        self.job_start_time = None
        self.job_duration = 0.0

    def _init_sensor_data(self):
        """Initialize sensor data containers"""
        self.battery_status = BatteryState()
        self.task = Task()
        self.gps_status = NavSatFix()
        self.pose_status = PoseWithCovarianceStamped()
        self.imu_status = Imu()
        self.estop_status = Bool()

    def _init_subscriptions(self):
        """Initialize all ROS subscriptions"""
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

    def _init_publishers(self):
        """Initialize all ROS publishers"""
        self.robot_state_pub = self.create_publisher(
            RobotStatus,
            f'{self.namespace}/status/robot',
            10)

    def _init_action_clients(self):
        """Initialize all action clients"""
        self.navigation = NavigationActionClient(self)
        self.april_tag_docking = DockingActionClient(self)
        self.april_tag_undocking = UndockingActionClient(self)

    # =========================================================================
    # MAIN CALLBACK METHODS
    # =========================================================================

    def _task_callback(self, msg: Task):
        """Handle incoming task messages"""
        is_new_task = msg.task_id != self.last_handled_task_id

        # Check if subtask changed
        is_new_subtask = False
        if isinstance(msg.sub_tasks, list) and len(msg.sub_tasks) > 0:
            first_subtask = msg.sub_tasks[0]
            if isinstance(first_subtask, SubTask):
                is_new_subtask = first_subtask.type != self.last_handled_subtask_type

        if is_new_task:
            self.get_logger().info(
                f"New Task: {msg.description} (ID: {msg.task_id})")
            self.current_sub_task_index = 0
            self.last_handled_subtask_type = None
        elif is_new_subtask:
            self.get_logger().info(f"New Subtask for task ID: {msg.task_id}")
            self.current_sub_task_index = 0
            if self.current_status == RobotStatusEnum.JOB_DONE:
                self._transition_status(RobotStatusEnum.IDLE)

        self.task = msg

    def timer_callback(self):
        """Main control loop - runs at configured frequency"""
        robot_status = RobotStatus()
        robot_status.header.stamp = self.get_clock().now().to_msg()
        robot_status.robot_namespace = self.namespace.replace(r'/', '')

        # Update sensor data
        self._set_battery_status(robot_status)
        self._set_estop_status(robot_status)
        self._set_location_status(robot_status)

        # Execute state machine
        if self.current_status in [RobotStatusEnum.ERROR, RobotStatusEnum.ABNORMAL]:
            self._handle_error_recovery()
        elif not self.startup_undock_complete:
            self._handle_startup_undocking(robot_status)
        else:
            self._handle_task_execution(robot_status)

        # Publish status
        robot_status.status = self.current_status.value
        robot_status.current_node_id = self.current_node_id
        robot_status.load_status = self.current_load_status
        self.robot_state_pub.publish(robot_status)

    # =========================================================================
    # STARTUP AND INITIALIZATION
    # =========================================================================

    def check_initial_position(self):
        """Check if robot starts at docking station"""
        if self.is_initialized:
            return

        if not self.pose_status or not self.pose_status.pose:
            self.get_logger().warning("No pose data - retrying in 1s")
            self.create_timer(1.0, self.check_initial_position, clock=self.get_clock())
            return

        current_pos = self.pose_status.pose.pose.position
        distance = self._calculate_distance(
            current_pos.x, current_pos.y,
            self.charging_x, self.charging_y)

        self.get_logger().info(
            f"Initial position: ({current_pos.x:.3f}, {current_pos.y:.3f}), "
            f"Distance to dock: {distance:.3f}m")

        if distance <= RobotConfig.DOCKING_THRESHOLD:
            self.is_at_docking_station = True
            self.get_logger().info("Robot at dock - will undock before tasks")
        else:
            self.is_at_docking_station = False
            self.startup_undock_complete = True
            self.get_logger().info("Robot ready for tasks")

        self.is_initialized = True

    def _handle_startup_undocking(self, robot_status: RobotStatus):
        """Execute startup undocking sequence if needed"""
        if not self.is_initialized or self.startup_undock_complete:
            return

        if self.current_status == RobotStatusEnum.IDLE:
            self.get_logger().info("Starting startup undocking...")
            self._transition_status(RobotStatusEnum.START_UNDOCKING)
            robot_status.task = "Startup: Preparing to undock"

        elif self.current_status == RobotStatusEnum.START_UNDOCKING:
            self._transition_status(RobotStatusEnum.UNDOCKING)
            robot_status.task = "Startup: Undocking"

            startup_subtask = SubTask()
            startup_subtask.type = SubTask.UNDOCKING
            startup_subtask.description = "Startup Undocking"
            startup_subtask.undock_goal = UndockGoal(
                dock_type='jackal_dock',
                max_undocking_time=30.0
            )

            self.april_tag_undocking.send_undocking_goal(startup_subtask)
        else:
            self._handle_undocking(robot_status)

        # elif self.current_status == RobotStatusEnum.UNDOCKING:
        #     robot_status.task = "Startup: Undocking in progress"

        # elif self.current_status == RobotStatusEnum.DONE_UNDOCKING:
        #     self.get_logger().info("Startup undocking complete")
        #     self.startup_undock_complete = True
        #     self._transition_status(RobotStatusEnum.IDLE)

    # =========================================================================
    # TASK EXECUTION
    # =========================================================================

    def _handle_task_execution(self, robot_status: RobotStatus):
        """Main task execution handler"""
        # Check for valid task
        if not self.task or not self.task.description or not self.task.job_schedule:
            if self.current_status != RobotStatusEnum.IDLE:
                self._transition_status(RobotStatusEnum.IDLE)
            return

        self.current_task = self.task

        # Check battery first
        if self._check_and_handle_low_battery():
            return

        # Process action client updates
        self._process_action_clients(robot_status)

        # Update current subtask
        self._update_current_subtask()

        # Update robot status message
        robot_status.crop_type = self.current_task.crop_type
        robot_status.target_node_id = self.current_task.target_node_id
        robot_status.task = (self.current_sub_task.description
                            if self.current_sub_task else self.current_task.description)

        # Execute state machine
        if self.current_status in (RobotStatusEnum.IDLE, RobotStatusEnum.JOB_DONE):
            self._handle_task_start()
        else:
            self._execute_current_subtask()

    def _check_and_handle_low_battery(self) -> bool:
        """Check battery and transition to error if low"""
        if not self.task or not self.current_task:
            return False

        battery_pct = self._normalize_battery(self.battery_status.percentage)

        # Low battery during non-charging task
        if (self.task.task_type != Task.CHARGING_TASK and
            battery_pct <= RobotConfig.LOW_BATTERY_THRESHOLD):

            self.get_logger().warning(f"Battery Low! {battery_pct:.1f}%")

            # Cancel navigation if active
            if self.current_status in [RobotStatusEnum.START_MOVING, RobotStatusEnum.MOVING]:
                self.get_logger().warning("Cancelling navigation - low battery")
                self.navigation.cancel_goal()

            self._transition_status(RobotStatusEnum.ERROR)
            return True

        # Recover from low battery error if charging task received
        if (self.current_status == RobotStatusEnum.ERROR and
            self.current_task.task_type == Task.CHARGING_TASK):
            self.get_logger().info("Recovering from low battery - charging task received")
            self._transition_status(RobotStatusEnum.IDLE)

        return False

    def _process_action_clients(self, robot_status: RobotStatus):
        """Process status updates from all action clients"""
        if self.navigation.get_navigation_status() != NavigationStatus.IDLE:
            self._handle_navigation(robot_status)
        elif self.april_tag_docking.get_status() != RobotStatusEnum.IDLE:
            self._handle_docking(robot_status)
        elif self.april_tag_undocking.get_status() != RobotStatusEnum.IDLE:
            self._handle_undocking(robot_status)

    def _update_current_subtask(self):
        """Update current subtask based on index"""
        if not self.current_task:
            return

        if isinstance(self.current_task.sub_tasks, list):
            if self.current_sub_task_index < len(self.current_task.sub_tasks):
                self.current_sub_task = self.current_task.sub_tasks[self.current_sub_task_index]
            else:
                self.current_sub_task = None

    def _handle_task_start(self):
        """Handle new task initialization"""
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
        """Route to appropriate subtask handler"""
        if not isinstance(self.current_sub_task, SubTask):
            return

        # Log new subtask
        if self.current_sub_task.type != self.last_handled_subtask_type:
            self.get_logger().info(
                f"Executing: {self.current_sub_task.description}")
            self.last_handled_subtask_type = self.current_sub_task.type

        # Route to handler
        task_handler_map = {
            SubTask.MOVING: self._subtask_moving,
            SubTask.HARVESTING: self._subtask_harvesting,
            SubTask.DOCKING: self._subtask_docking,
            # SubTask.UNDOCKING: self._subtask_undocking,
            SubTask.LOADING: self._subtask_loading,
            SubTask.CHARGING: self._subtask_charging,
            SubTask.UNLOADING: self._subtask_unloading,
        }

        handler = task_handler_map.get(self.current_sub_task.type)
        if handler:
            handler()
        else:
            self.get_logger().warning(
                f"Unknown subtask type: {self.current_sub_task.type}")

    # =========================================================================
    # ACTION CLIENT HANDLERS
    # =========================================================================

    def _handle_navigation(self, robot_status: RobotStatus):
        """Handle navigation action client status"""
        nav_status = self.navigation.get_navigation_status()
        wpf_status = self.navigation.get_current_status()

        # Update robot status
        if wpf_status:
            robot_status.task = wpf_status.task
            robot_status.current_node_id = wpf_status.current_node_id
            robot_status.target_node_id = wpf_status.target_node_id
        else:
            robot_status.task = self.current_task.description if self.current_task else ""
            robot_status.current_node_id = self.current_node_id
            robot_status.target_node_id = (
                self.current_task.target_node_id if self.current_task else -1)

        # Handle status
        if nav_status == NavigationStatus.ACTIVE:
            self._transition_status(RobotStatusEnum.MOVING)

        elif nav_status == NavigationStatus.SUCCEEDED:
            self.get_logger().info("Navigation complete")
            self.navigation_retry_count = 0
            self.navigation.reset()
            self.current_node_id = (wpf_status.target_node_id
                                   if wpf_status else self.current_node_id)
            self._transition_status(RobotStatusEnum.DESTINATION_REACHED)

        elif nav_status in [NavigationStatus.ABORTED, NavigationStatus.ERROR]:
            self._handle_navigation_retry()

        elif nav_status == NavigationStatus.CANCELED:
            self.get_logger().info("Navigation canceled")
            self.navigation.reset()
            self.navigation_retry_count = 0
            self._transition_status(RobotStatusEnum.IDLE)

    def _handle_navigation_retry(self):
        """Handle navigation failure with retry logic"""
        self.get_logger().warning(
            f"Navigation failed (retry {self.navigation_retry_count + 1}/"
            f"{RobotConfig.MAX_NAVIGATION_RETRIES})")

        if self.navigation_retry_count < RobotConfig.MAX_NAVIGATION_RETRIES:
            self.navigation_retry_count += 1
            self.get_logger().info(
                f"Retrying navigation in {RobotConfig.NAVIGATION_RETRY_DELAY:.1f}s")

            self.create_timer(
                RobotConfig.NAVIGATION_RETRY_DELAY,
                self._retry_navigation,
                clock=self.get_clock())
        else:
            self.get_logger().error("Navigation failed after max retries")
            self.navigation.reset()
            self._transition_status(RobotStatusEnum.ERROR)
            self.navigation_retry_count = 0

    def _retry_navigation(self):
        """Retry navigation goal"""
        if not self.current_task:
            self.get_logger().error("No task for navigation retry")
            return

        self.get_logger().info(
            f"Retrying navigation (attempt {self.navigation_retry_count}/"
            f"{RobotConfig.MAX_NAVIGATION_RETRIES})")

        if self.navigation.send_goal(self.current_task):
            self._transition_status(RobotStatusEnum.MOVING)
        else:
            self.get_logger().error("Retry failed")
            self._transition_status(RobotStatusEnum.ERROR)

    def _handle_docking(self, robot_status: RobotStatus):
        """Handle docking action client status"""
        status = self.april_tag_docking.get_status()
        feedback = self.april_tag_docking.get_feedback()

        # Update robot status
        if feedback:
            robot_status.task = feedback.task
            robot_status.current_node_id = self.current_node_id
        else:
            robot_status.task = "Docking in progress"

        # Handle status
        if status == RobotStatusEnum.DOCKING:
            self._transition_status(RobotStatusEnum.DOCKING)

        elif status == RobotStatusEnum.DONE_DOCKING:
            self.get_logger().info("Docking complete")
            self._transition_status(RobotStatusEnum.DONE_DOCKING)
            self.april_tag_docking.reset()
            self.docking_retry_count = 0

        elif status == RobotStatusEnum.ERROR:
            self._handle_docking_retry()

    def _handle_docking_retry(self):
        """Handle docking failure with retry logic"""
        self.get_logger().error(
            f"Docking failed (retry {self.docking_retry_count + 1}/"
            f"{RobotConfig.MAX_DOCKING_RETRIES})")

        if self.docking_retry_count < RobotConfig.MAX_DOCKING_RETRIES:
            self.docking_retry_count += 1
            self.get_logger().info(
                f"Retrying docking in {RobotConfig.DOCKING_RETRY_DELAY:.1f}s")

            self.create_timer(
                RobotConfig.DOCKING_RETRY_DELAY,
                self._retry_docking,
                clock=self.get_clock())
        else:
            self.get_logger().error("Docking failed after max retries")
            self.april_tag_docking.reset()
            self._transition_status(RobotStatusEnum.ERROR)
            self.docking_retry_count = 0

    def _retry_docking(self):
        """Retry docking goal"""
        if not self.current_sub_task:
            self.get_logger().warning("No subtask for docking retry")
            return

        if self.april_tag_docking.send_docking_goal(self.current_sub_task):
            self.get_logger().info(f"Reattempting docking (try #{self.docking_retry_count})")
            self._transition_status(RobotStatusEnum.DOCKING)
        else:
            self.get_logger().error("Docking retry failed")
            self._transition_status(RobotStatusEnum.ERROR)

    def _handle_undocking(self, robot_status: RobotStatus):
        """Handle undocking action client status"""
        status = self.april_tag_undocking.get_status()
        feedback = self.april_tag_undocking.get_feedback()

        # Update robot status
        if feedback:
            robot_status.task = feedback.task
            robot_status.current_node_id = self.current_node_id
        else:
            robot_status.task = "Undocking in progress"

        # Handle status
        if status == RobotStatusEnum.UNDOCKING:
            self._transition_status(RobotStatusEnum.UNDOCKING)

        elif status == RobotStatusEnum.DONE_UNDOCKING:
            self.get_logger().info("Undocking complete")
            self._transition_status(RobotStatusEnum.DONE_UNDOCKING)
            self.april_tag_undocking.reset()
            self.undocking_retry_count = 0
            self.startup_undock_complete = True
            self._transition_status(RobotStatusEnum.IDLE)

        elif status == RobotStatusEnum.ERROR:
            self._handle_undocking_retry()

    def _handle_undocking_retry(self):
        """Handle undocking failure with retry logic"""
        self.get_logger().error(
            f"Undocking failed (retry {self.undocking_retry_count + 1}/"
            f"{RobotConfig.MAX_UNDOCKING_RETRIES})")

        if self.undocking_retry_count < RobotConfig.MAX_UNDOCKING_RETRIES:
            self.undocking_retry_count += 1
            self.get_logger().info(
                f"Retrying undocking in {RobotConfig.UNDOCKING_RETRY_DELAY:.1f}s")

            self.create_timer(
                RobotConfig.UNDOCKING_RETRY_DELAY,
                self._retry_undocking,
                clock=self.get_clock())
        else:
            self.get_logger().error("Undocking failed after max retries")
            self.april_tag_undocking.reset()
            self._transition_status(RobotStatusEnum.ERROR)
            self.undocking_retry_count = 0

    def _retry_undocking(self):
        """Retry undocking goal"""
        if not self.current_sub_task:
            self.get_logger().warning("No subtask for undocking retry")
            return

        if self.april_tag_undocking.send_undocking_goal(self.current_sub_task):
            self.get_logger().info(f"Reattempting undocking (try #{self.undocking_retry_count})")
            self._transition_status(RobotStatusEnum.UNDOCKING)
        else:
            self.get_logger().error("Undocking retry failed")
            self._transition_status(RobotStatusEnum.ERROR)

    # =========================================================================
    # ERROR HANDLING
    # =========================================================================

    def _handle_error_recovery(self):
        """Handle error state recovery"""
        if self.current_status not in [RobotStatusEnum.ERROR, RobotStatusEnum.ABNORMAL]:
            return

        self.get_logger().warning(f"Error recovery - status: {self.current_status.name}")

        # Cancel any active operations
        if self.navigation.is_navigation_active():
            self.get_logger().info("Cancelling navigation")
            self.navigation.cancel_goal()

        # Reset state
        self.current_sub_task = None
        self.current_sub_task_index = 0

        # Transition to idle
        self.get_logger().info("Transitioning to IDLE")
        self._transition_status(RobotStatusEnum.IDLE)

    # =========================================================================
    # SUBTASK HANDLERS
    # =========================================================================

    def _subtask_moving(self):
        """Handle moving subtask"""
        if not self.current_task:
            return

        if self.current_status == RobotStatusEnum.JOB_START:
            self._transition_status(RobotStatusEnum.START_MOVING)

        elif self.current_status == RobotStatusEnum.START_MOVING:
            if self.navigation.is_navigation_active():
                self.get_logger().warning("Navigation already active")
                return

            self.get_logger().info(
                f"Starting navigation: {self.current_node_id} → "
                f"{self.current_task.target_node_id}")

            if self.navigation.send_goal(self.current_task):
                self._transition_status(RobotStatusEnum.MOVING)
            else:
                self.get_logger().error("Failed to send navigation goal")
                self._transition_status(RobotStatusEnum.ERROR)

    def _subtask_docking(self):
        """Handle docking subtask"""
        if self.current_status == RobotStatusEnum.DESTINATION_REACHED:
            self._transition_status(RobotStatusEnum.START_DOCKING)

        elif self.current_status == RobotStatusEnum.START_DOCKING:
            self.get_logger().info("Starting docking")

            if self.april_tag_docking.send_docking_goal(self.current_sub_task):
                self._transition_status(RobotStatusEnum.DOCKING)
            else:
                self.get_logger().error("Failed to send docking goal")
                self._transition_status(RobotStatusEnum.ERROR)

    def _subtask_undocking(self):
        """Handle undocking subtask"""
        # if self.current_status == RobotStatusEnum.DONE_DOCKING:
        #     self._transition_status(RobotStatusEnum.START_UNDOCKING)

        if self.current_status == RobotStatusEnum.START_UNDOCKING:
            self.get_logger().info("Starting undocking")

            if self.april_tag_undocking.send_undocking_goal(self.current_sub_task):
                self._transition_status(RobotStatusEnum.UNDOCKING)
            else:
                self.get_logger().error("Failed to send undocking goal")
                self._transition_status(RobotStatusEnum.ERROR)

        elif self.current_status == RobotStatusEnum.DONE_UNDOCKING:
            self.get_logger().info("Undocking done")
            self._transition_status(RobotStatusEnum.JOB_DONE)

    def _subtask_charging(self):
        """Handle charging subtask"""
        if self.current_status == RobotStatusEnum.DONE_DOCKING:
            self._transition_status(RobotStatusEnum.START_CHARGING)

        elif self.current_status == RobotStatusEnum.START_CHARGING:
            self._transition_status(RobotStatusEnum.CHARGING)
            self.get_logger().info("Charging started")

        elif self.current_status == RobotStatusEnum.CHARGING:
            battery_pct = self._normalize_battery(self.battery_status.percentage)

            if battery_pct >= RobotConfig.FULL_BATTERY_THRESHOLD:
                self.get_logger().info(f"Battery charged: {battery_pct:.1f}%")
                self._transition_status(RobotStatusEnum.DONE_CHARGING)

        elif self.current_status == RobotStatusEnum.DONE_CHARGING:
            self._transition_status(RobotStatusEnum.START_UNDOCKING)
            self._subtask_undocking()

    def _subtask_harvesting(self):
        """
        Handle harvesting subtask

        TODO: Integrate with actual harvesting hardware/controller
        - Connect to harvesting mechanism controller
        - Monitor harvesting sensors (crop detection, bin status)
        - Handle harvesting completion signal
        - Add error handling for harvesting failures
        """
        if self.current_status == RobotStatusEnum.DESTINATION_REACHED:
            self._transition_status(RobotStatusEnum.START_HARVESTING)

        elif self.current_status == RobotStatusEnum.START_HARVESTING:
            self._transition_status(RobotStatusEnum.HARVESTING)
            self.get_logger().info("Harvesting started")

            # TODO: Send command to harvesting controller
            # self.harvesting_controller.start()

            # Start job timer for simulation
            self.job_start_time = self.get_clock().now()
            self.job_duration = 5.0  # Simulate 5 second harvest

        elif self.current_status == RobotStatusEnum.HARVESTING:
            # TODO: Check actual harvesting status
            # if self.harvesting_controller.is_complete():
            #     self._transition_status(RobotStatusEnum.DONE_HARVESTING)

            # PLACEHOLDER: Simulate harvesting with timer
            if self.job_start_time:
                elapsed = (self.get_clock().now() - self.job_start_time).nanoseconds / 1e9
                if elapsed >= self.job_duration:
                    self.get_logger().info("Harvesting complete (simulated)")
                    self._transition_status(RobotStatusEnum.DONE_HARVESTING)
                    self.job_start_time = None

    def _subtask_loading(self):
        """
        Handle loading subtask

        TODO: Integrate with actual loading hardware/controller
        - Connect to conveyor/loading mechanism
        - Monitor load sensors (weight, volume)
        - Handle loading completion signal
        - Update actual load status from sensors
        - Add error handling for loading failures
        """
        if self.current_status == RobotStatusEnum.DONE_HARVESTING:
            self._transition_status(RobotStatusEnum.START_LOADING)

        elif self.current_status == RobotStatusEnum.START_LOADING:
            self._transition_status(RobotStatusEnum.LOADING)
            self.get_logger().info("Loading started")

            # TODO: Send command to loading controller
            # self.loading_controller.start()

            # Start job timer for simulation
            self.job_start_time = self.get_clock().now()
            self.job_duration = 3.0  # Simulate 3 second load

        elif self.current_status == RobotStatusEnum.LOADING:
            # TODO: Check actual loading status
            # if self.loading_controller.is_complete():
            #     actual_load = self.loading_controller.get_load_percentage()
            #     self.current_load_status = actual_load
            #     self._transition_status(RobotStatusEnum.DONE_LOADING)

            # PLACEHOLDER: Simulate loading with timer
            if self.job_start_time:
                elapsed = (self.get_clock().now() - self.job_start_time).nanoseconds / 1e9
                if elapsed >= self.job_duration:
                    self.get_logger().info("Loading complete (simulated)")
                    self._transition_status(RobotStatusEnum.DONE_LOADING)
                    self.job_start_time = None

        elif self.current_status == RobotStatusEnum.DONE_LOADING:
            # Update load status (simulated increment)
            self.current_load_status = min(
                self.current_load_status + RobotConfig.LOAD_INCREMENT, 100.0)
            self.get_logger().info(f"Load status: {self.current_load_status:.1f}%")
            self._transition_status(RobotStatusEnum.JOB_DONE)

    def _subtask_unloading(self):
        """
        Handle unloading subtask

        TODO: Integrate with actual unloading hardware/controller
        - Connect to unloading mechanism
        - Monitor unload sensors (confirmation sensors)
        - Handle unloading completion signal
        - Verify complete unload before proceeding
        - Add error handling for unloading failures
        """
        if self.current_status == RobotStatusEnum.DONE_DOCKING:
            self._transition_status(RobotStatusEnum.START_UNLOADING)

        elif self.current_status == RobotStatusEnum.START_UNLOADING:
            self._transition_status(RobotStatusEnum.UNLOADING)
            self.get_logger().info("Unloading started")

            # TODO: Send command to unloading controller
            # self.unloading_controller.start()

            # Start job timer for simulation
            self.job_start_time = self.get_clock().now()
            self.job_duration = 4.0  # Simulate 4 second unload

        elif self.current_status == RobotStatusEnum.UNLOADING:
            # TODO: Check actual unloading status
            # if self.unloading_controller.is_complete():
            #     self._transition_status(RobotStatusEnum.DONE_UNLOADING)

            # PLACEHOLDER: Simulate unloading with timer
            if self.job_start_time:
                elapsed = (self.get_clock().now() - self.job_start_time).nanoseconds / 1e9
                if elapsed >= self.job_duration:
                    self.get_logger().info("Unloading complete (simulated)")
                    self._transition_status(RobotStatusEnum.DONE_UNLOADING)
                    self.job_start_time = None

        elif self.current_status == RobotStatusEnum.DONE_UNLOADING:
            self.current_load_status = 0.0
            self.get_logger().info("Unloading done, starting undocking")
            self._transition_status(RobotStatusEnum.START_UNDOCKING)
            self._subtask_undocking()

    # =========================================================================
    # SENSOR STATUS METHODS
    # =========================================================================

    def _set_battery_status(self, robot_status: RobotStatus):
        """Update battery status information"""
        robot_status.battery_level = self.battery_status.percentage

        if self.battery_status.capacity > 0.0 and self.battery_status.current > 0.0:
            battery_pct = self._normalize_battery(self.battery_status.percentage)
            time_remaining = (
                self.battery_status.capacity * (battery_pct / 100.0) /
                self.battery_status.current
            )
            robot_status.operation_hours_after_charging = self._format_time_remaining(
                time_remaining)
        else:
            robot_status.operation_hours_after_charging = (
                "00 hours 00 minutes remaining approx...")

    def _set_estop_status(self, robot_status: RobotStatus):
        """Update emergency stop status"""
        robot_status.online_flag = (
            self.estop_status.data if self.estop_status.data
            else OnlineFlagEnum.ONLINE.value
        )

    def _set_location_status(self, robot_status: RobotStatus):
        """Update robot location from pose data"""
        if self.pose_status and self.pose_status.pose:
            robot_status.topo_map_position = self.pose_status.pose.pose.position
            robot_status.topo_map_orientation = self.pose_status.pose.pose.orientation

    # =========================================================================
    # UTILITY METHODS
    # =========================================================================

    def _transition_status(self, new_status: RobotStatusEnum):
        """Safely transition to new status with logging"""
        if self.current_status != new_status:
            self.previous_status = self.current_status
            self.current_status = new_status
            self.get_logger().info(
                f"Status: {self.previous_status.name} → {self.current_status.name}")

    def _normalize_battery(self, percentage: float) -> float:
        """
        Normalize battery percentage to 0-100 range
        Handles both 0.0-1.0 and 0-100 representations
        """
        return percentage * 100.0 if percentage <= 1.0 else percentage

    def _format_time_remaining(self, hours: float) -> str:
        """Convert hours to formatted string (HH hours MM minutes)"""
        seconds = int(hours * 3600)
        minutes, seconds = divmod(seconds, 60)
        hours, minutes = divmod(minutes, 60)
        return f"{hours:02} hours {minutes:02} minutes remaining approximately."

    def _calculate_distance(self, x1: float, y1: float, x2: float, y2: float) -> float:
        """Calculate Euclidean distance between two points"""
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

    def _scale_value(self, value: float, original_min: float, original_max: float,
                   target_min: float, target_max: float) -> float:
        """Scale value from one range to another"""
        if original_max == original_min:
            raise ValueError("Original range cannot be zero.")

        return (((value - original_min) / (original_max - original_min)) *
                (target_max - target_min) + target_min)

    def _lat_lon_to_cartesian(self, lat: float, lon: float, alt: float) -> tuple:
        """Convert lat/lon/alt to Cartesian coordinates"""
        wgs84 = pyproj.CRS("EPSG:4326")
        geocentric = pyproj.CRS("EPSG:4978")
        transformer = pyproj.Transformer.from_crs(wgs84, geocentric, always_xy=True)
        return transformer.transform(lon, lat, alt)


# =============================================================================
# MAIN ENTRY POINT
# =============================================================================
def main(args=None):
    """Main entry point for the status node"""
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
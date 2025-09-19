import json
import pyproj
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, NavSatFix, Imu
from tf_transformations import euler_from_quaternion
from tf2_ros import Buffer, TransformListener

from geometry_msgs.msg import TransformStamped, Pose
from nav_msgs.msg import Odometry as Odom
from std_msgs.msg import Bool, String
from status_interfaces.msg import RobotStatus, Task, SubTask, WayPoint

from jackel_ops.april_tag_docking import DockingActionClient
from jackel_ops.april_tag_undocking import UndockingActionClient
from jackel_ops.navigate_through_pose import NavigationActionClientUsingNTP
from jackel_ops.dataclass import DockingFeedback, WPFStatus
from jackel_ops.enum import OnlineFlagEnum, RobotStatusEnum


class StatusNode(Node):
    def __init__(self):
        super().__init__('status_node')
        self.get_logger().info('StatusNode has been started.')

        # Current state
        self.current_status = RobotStatusEnum.IDLE
        self.current_task: Task
        self.current_sub_task = None
        self.current_sub_task_list = None
        self.current_node_id = 0
        self.current_load_status = 0.0
        self.last_handled_task_id = None

        # Subscriptions messages
        self.battery_status: BatteryState = BatteryState()
        self.task: Task = Task()
        self.gps_status: NavSatFix = NavSatFix()
        # self.odom_status: Odom = Odom()
        self.pose_status: Pose = Pose()
        self.imu_status: Imu = Imu()
        self.estop_status: Bool = Bool()
        self.wpf_status: String = String()
        self.docking_status: String = String()
        self.undocking_status: String = String()

        # Setting namespace for Node
        self.namespace = self.get_namespace().rstrip('/')

        self.tf2_buffer = Buffer()
        self.listener = TransformListener(self.tf2_buffer, self)

        # Timer to periodically check robot pose
        self.tf2_timer = self.create_timer(1.0, self.get_robot_pose)

        # Subscriptions
        # Battery Topic
        self.battery_sub = self.create_subscription(
            BatteryState,
            f'{self.namespace}/platform/bms/state',
            lambda msg: setattr(self, 'battery_status', msg),
            qos_profile_sensor_data)

        # # NavSat Topic
        # self.navsat_sub = self.create_subscription(
        #     NavSatFix,
        #     f'{self.namespace}/sensors/gps_0/nmea_sentence',
        #     lambda msg: setattr(self, 'gps_status', msg),
        #     qos_profile_sensor_data)

        # IMU Topic
        self.imu_sub = self.create_subscription(
            Imu,
            f'{self.namespace}/sensors/imu_0/magnetic_field',
            lambda msg: setattr(self, 'imu_status', msg),
            qos_profile_sensor_data)

        # E-Stop Topic
        self.estop_sub = self.create_subscription(
            Bool,
            f'{self.namespace}/platform/emergency_stop',
            lambda msg: setattr(self, 'estop_status', msg),
            qos_profile_sensor_data)

        self.task_sub = self.create_subscription(
            Task,
            f'{self.namespace}/status/task',
            lambda msg: setattr(self, 'task', msg),
            10)

        self.wpf_sub = None
        self.docking_sub = None
        self.undocking_sub = None

        # Publishers
        # Robot Status
        self.robot_state_pub = self.create_publisher(
            RobotStatus, f'{self.namespace}/status/robot', 10)

        self.timer = self.create_timer(1.0, self.timer_callback)

        # Action Clients
        self.navigation = NavigationActionClientUsingNTP(self)
        self.april_tag_docking = DockingActionClient(self)
        self.april_tag_undocking = UndockingActionClient(self)

    def get_robot_pose(self):
        try:
            # Lookup transform from map -> base_link
            now = Time()
            transform: TransformStamped = self.tf2_buffer.lookup_transform(
                'map',        # target frame
                'base_link',  # source frame
                now,
                timeout=Duration(seconds=1)
            )

            # Extract translation (robot position)
            self.pose_status.position.x = transform.transform.translation.x
            self.pose_status.position.y = transform.transform.translation.y
            self.pose_status.position.z = transform.transform.translation.z

            # Extract orientation quaternion
            self.pose_status.orientation = transform.transform.rotation
            # self.get_logger().info(
            #     f"Robot pose: x={x:.3f}, y={y:.3f}, z={z:.3f}, "
            #     f"quat=({q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f})"
            # )

        except Exception as e:
            self.get_logger().warn(f"Could not get transform: {e}")

    def timer_callback(self):
        robot_status = RobotStatus()

        robot_status.header.stamp = self.get_clock().now().to_msg()
        robot_status.robot_namespace = self.namespace.replace(r'/', '')

        self.set_battery_status(robot_status)
        self.set_estop_status(robot_status)
        self.set_location_status(robot_status)

        self.handle_task(robot_status)

        robot_status.status = self.current_status.value
        robot_status.current_node_id = self.current_node_id
        robot_status.load_status = self.current_load_status
        self.robot_state_pub.publish(robot_status)

    def handle_task(self, robot_status: RobotStatus):
        # Check for the received task
        if self.task.description == "" or self.task.job_schedule == "":
            self.current_status = RobotStatusEnum.IDLE
            # self.get_logger().info(self.task_status)
            self.get_logger().info(f"No Task received for the robot {self.namespace}")
            return

        # # Converting string to dataclass for Task
        # data_dict = json.loads(self.task_status.data)
        # task = Task(**data_dict)
        self.current_task = self.task
        # self.get_logger().info(f"Current Task: {self.current_task.description}")

        # #Navigation Status from Action Client
        if self.wpf_sub is not None and self.wpf_status is not None:
            self.handle_way_point_follower(robot_status)

        # #Docking Status from Action Client
        elif self.docking_sub is not None and self.docking_status is not None:
            self.handle_docking(robot_status)

        # #Undocking Status from Action Client
        elif self.undocking_sub is not None and self.undocking_status is not None:
            self.handle_undocking(robot_status)

        # self.get_logger().info(f"Sub-Task: { type(self.current_task.sub_tasks)}")

        # Converting string to dataclass for Sub Task
        if isinstance(self.current_task.sub_tasks, list):
            # self.get_logger().info(f"Sub-Task 1: {self.current_task.sub_tasks}")
            self.current_sub_task = self.current_task.sub_tasks[0]

        # #Crop Type
        robot_status.crop_type = self.current_task.crop_type

        # Updating task description based on the status
        if self.current_sub_task is not None:
            robot_status.task = self.current_sub_task.description
        elif self.current_status == RobotStatusEnum.IDLE:
            robot_status.task = self.current_task.description

        robot_status.target_node_id = self.current_task.target_node_id

        # self.get_logger().info(
        #     f"Current Task1: {self.current_task.description}")

        # Charging task for battery below 50% and publish status as error
        if (self.task.task_type != Task.CHARGING_TASK and
                self.battery_status.percentage <= 0.5):
            self.get_logger().warning("Battery Low!!!!!!!")

            if self.current_status in [RobotStatusEnum.START_MOVING, RobotStatusEnum.MOVING]:
                self.navigation.cancel_goal()

            self.current_status = RobotStatusEnum.ERROR
            return

        # Receiving charging task and updating the error state generated due to low battery
        if (self.current_status == RobotStatusEnum.ERROR and
                self.current_task.task_type == Task.CHARGING_TASK):
            if self.wpf_sub is not None and self.docking_sub is not None:
                self.destroy_subscription(self.wpf_sub)
                self.destroy_subscription(self.docking_sub)

            self.current_status = RobotStatusEnum.IDLE

        if self.current_status in (RobotStatusEnum.IDLE, RobotStatusEnum.JOB_DONE):
            # Updating current status of robot if new task is received
            self.current_status = (
                RobotStatusEnum.JOB_START if self.current_status == RobotStatusEnum.IDLE
                else RobotStatusEnum.IDLE)
        else:
            self.handle_job_start()

    # region Action Client Status
    # #Navigation
    def handle_way_point_follower(self, robot_status: RobotStatus):
        if self.wpf_status.data != '' and self.wpf_status.data is not None:

            data_dict = json.loads(self.wpf_status.data)
            self.wpf_status_data = WPFStatus(**data_dict)

            robot_status.task = self.wpf_status_data.task
            robot_status.target_node_id = self.current_task.target_node_id
            robot_status.current_node_id = self.wpf_status_data.current_node_id

            status = self.wpf_status_data.status

            status_map = {
                RobotStatusEnum.MOVING.value: RobotStatusEnum.MOVING,
                RobotStatusEnum.DESTINATION_REACHED.value: RobotStatusEnum.DESTINATION_REACHED,
                RobotStatusEnum.ERROR.value: RobotStatusEnum.ERROR,
            }

            self.current_status = status_map.get(status, self.current_status)

            if status in (RobotStatusEnum.DESTINATION_REACHED.value, RobotStatusEnum.ERROR.value):
                if self.wpf_sub is not None:
                    self.destroy_subscription(self.wpf_sub)

                self.wpf_sub = None

            self.current_node_id = self.wpf_status_data.current_node_id

    # #Docking
    def handle_docking(self, robot_status: RobotStatus):
        # self.get_logger().info(
        #     f"Docking Status: {self.docking_status.data}")

        if self.docking_status.data != '' and self.docking_status.data is not None:

            data_dict = json.loads(self.docking_status.data)
            status_data = DockingFeedback(**data_dict)

            robot_status.task = status_data.task
            robot_status.target_node_id = self.current_task.target_node_id
            robot_status.current_node_id = self.current_node_id

            status = status_data.status

            status_map = {
                RobotStatusEnum.DOCKING.value: RobotStatusEnum.DOCKING,
                RobotStatusEnum.DONE_DOCKING.value: RobotStatusEnum.DONE_DOCKING,
                RobotStatusEnum.ERROR.value: RobotStatusEnum.ERROR,
            }

            self.current_status = status_map.get(status, self.current_status)

            # if status in (RobotStatusEnum.DONE_DOCKING.value, RobotStatusEnum.ERROR.value):
            if status == RobotStatusEnum.DONE_DOCKING.value:
                if self.docking_sub is not None:
                    self.destroy_subscription(self.docking_sub)

                self.docking_sub = None

            self.current_node_id = self.current_task.target_node_id

    # #Undocking
    def handle_undocking(self, robot_status: RobotStatus):
        # self.get_logger().info(
        #     f"Undocking Status: {self.docking_status.data}")

        if self.undocking_status.data != '' and self.undocking_status.data is not None:

            data_dict = json.loads(self.undocking_status.data)
            status_data = DockingFeedback(**data_dict)

            robot_status.task = status_data.task
            robot_status.target_node_id = self.current_task.target_node_id
            robot_status.current_node_id = self.current_node_id

            status = status_data.status

            self.get_logger().info(f"Undocking status: {status}")

            status_map = {
                RobotStatusEnum.UNDOCKING.value: RobotStatusEnum.UNDOCKING,
                RobotStatusEnum.DONE_UNDOCKING.value: RobotStatusEnum.DONE_UNDOCKING,
                RobotStatusEnum.ERROR.value: RobotStatusEnum.ERROR,
            }

            self.current_status = status_map.get(status, self.current_status)

            if status in (RobotStatusEnum.DONE_UNDOCKING.value, RobotStatusEnum.ERROR.value):
                if self.undocking_sub is not None:
                    self.destroy_subscription(self.undocking_sub)

                self.undocking_sub = None

                self.current_node_id = self.current_task.target_node_id

            if self.current_status == RobotStatusEnum.DONE_UNDOCKING:
                self.current_status = RobotStatusEnum.JOB_DONE
    # endregion

    # region TASKS
    def handle_job_start(self):
        if not isinstance(self.current_sub_task, SubTask):
            return

        if self.current_task.task_id != self.last_handled_task_id:
            self.get_logger().info(
                f"Found Task: {self.current_task.description} |"
                f"Current Node id: {self.current_node_id} | "
                f"Target Node id: {self.current_task.target_node_id}")
            # self.get_logger().info(
            #     f"Sub Task: {self.current_task.sub_tasks}")
            self.last_handled_task_id = self.current_task.task_id

        task_methods = {
            SubTask.MOVING: self.sub_task_moving,
            SubTask.HARVESTING: self.sub_task_harvesting,
            SubTask.DOCKING: self.sub_task_docking,
            SubTask.LOADING: self.sub_task_loading,
            SubTask.CHARGING: self.sub_task_charging,
            SubTask.UNLOADING: self.sub_task_unloading,
            # SubTask.UNDOCKING: self.sub_task_undocking,
        }
        task_methods.get(self.current_sub_task.type, lambda: None)()

    # region SUB-TASKS
    def sub_task_charging(self):
        # Start
        if self.current_status == RobotStatusEnum.DONE_DOCKING:
            self.current_status = RobotStatusEnum.START_CHARGING
        # Start
        if self.current_status == RobotStatusEnum.START_CHARGING:
            self.current_status = RobotStatusEnum.CHARGING
        # In-progress
        elif self.current_status == RobotStatusEnum.CHARGING:

            self.get_logger().info(
                f"Battery Charging: {self.battery_status.percentage}")

            # UPDATE CHARGING STATUS HERE
            if self.battery_status.percentage == 100.0 or self.battery_status.percentage == 1.0:
                self.current_status = RobotStatusEnum.DONE_CHARGING

                # self.get_logger().info(
                #     f"Current Charging Sub task: {self.current_sub_task}")
                # self.sub_task_undocking()
        # Done
        elif self.current_status == RobotStatusEnum.DONE_CHARGING:
            self.current_status = RobotStatusEnum.START_UNDOCKING
            self.sub_task_undocking()

    def sub_task_docking(self):
        if self.current_status == RobotStatusEnum.DESTINATION_REACHED:
            self.current_status = RobotStatusEnum.START_DOCKING
        # Start
        if self.current_status == RobotStatusEnum.START_DOCKING:
            self.current_status = RobotStatusEnum.DOCKING

            self.docking_sub = self.create_subscription(
                String,
                f'{self.namespace}/status/robot/docking',
                lambda msg: setattr(self, 'docking_status', msg),
                10)

            self.april_tag_docking.send_docking_goal(self.current_sub_task)

    def sub_task_undocking(self):
        # self.get_logger().info("Inside Undocking")
        # self.get_logger().info(
        #     f"Current Status Undocking: {self.current_status}")

        if self.current_status == RobotStatusEnum.START_UNDOCKING:
            self.current_status = RobotStatusEnum.UNDOCKING
            # self.get_logger().info(
            #     f"Current Status Undocking 2: {self.current_status}")
        # elif self. current_status == RobotStatusEnum.START_UNDOCKING:
        #     self.current_status = RobotStatusEnum.UNDOCKING

            self.get_logger().info("Sending Undocking Goal!!!!!!!!!!!!!!")
            self.undocking_sub = self.create_subscription(
                String,
                f'{self.namespace}/status/robot/undocking',
                lambda msg: setattr(self, 'undocking_status', msg),
                # self.undockingCallback
                10)

            self.april_tag_undocking.send_undocking_goal(self.current_sub_task)

            return

            # if isinstance(self.current_sub_task, SubTask):
            #     data = json.dumps(asdict(self.current_sub_task))
            #     msg = String()
            #     msg.data = data
            #     self.undocking_pub.publish(msg)

            #     self.get_logger().info(
            #         f"Publishing Undocking Task: {msg}")

    # def undockingCallback(self, msg):
    #     print("Received undocking status!")
    #     self.undocking_status = msg.data

    def sub_task_harvesting(self):
        # Done sub task moving
        if self.current_status == RobotStatusEnum.DESTINATION_REACHED:
            self.current_status = RobotStatusEnum.START_HARVESTING
            self.get_logger().info(f"harvesting: {self.current_status}")
        # Start
        elif self.current_status == RobotStatusEnum.START_HARVESTING:
            self.current_status = RobotStatusEnum.HARVESTING
            self.get_logger().info(f"harvesting: {self.current_status}")
        # In-progress
        elif self.current_status == RobotStatusEnum.HARVESTING:

            # UPDATE HARVESTING STATUS HERE

            self.current_status = RobotStatusEnum.DONE_HARVESTING
            self.get_logger().info(f"harvesting: {self.current_status}")

    def sub_task_loading(self):
        # Done
        if self.current_status == RobotStatusEnum.DONE_HARVESTING:
            self.current_status = RobotStatusEnum.START_LOADING
            self.get_logger().info(f"loading: {self.current_status}")
        # Start
        elif self.current_status == RobotStatusEnum.START_LOADING:
            self.current_status = RobotStatusEnum.LOADING
            self.get_logger().info(f"loading: {self.current_status}")
        # In-progress
        elif self.current_status == RobotStatusEnum.LOADING:

            # UPDATE LOADING STATUS HERE
            self.current_status = RobotStatusEnum.DONE_LOADING
            self.get_logger().info(f"loading: {self.current_status}")
        # Done
        elif self.current_status == RobotStatusEnum.DONE_LOADING:
            if self.current_load_status < 100:
                self.current_load_status += 20.0
            self.current_status = RobotStatusEnum.JOB_DONE
            self.get_logger().info(f"loading: {self.current_status}")

    def sub_task_moving(self):
        # Start
        if self.current_status == RobotStatusEnum.JOB_START:
            self.current_status = RobotStatusEnum.START_MOVING
        # In-progress
        elif self.current_status == RobotStatusEnum.START_MOVING:
            self.current_status = RobotStatusEnum.MOVING

            # self.navigation.is_goal_cancelled = False
            self.wpf_sub = self.create_subscription(
                String,
                f'{self.namespace}/status/robot/navigation',
                lambda msg: setattr(self, 'wpf_status', msg),
                10)
            self.navigation.send_goal(self.current_task)

    def sub_task_unloading(self):
        # Start
        if self.current_status == RobotStatusEnum.DONE_DOCKING:
            self.current_status = RobotStatusEnum.START_UNLOADING
            self.get_logger().info(f"unloading: {self.current_status}")
        elif self.current_status == RobotStatusEnum.START_UNLOADING:
            self.current_status = RobotStatusEnum.UNLOADING
            self.get_logger().info(f"unloading: {self.current_status}")
        # In-progress
        elif self.current_status == RobotStatusEnum.UNLOADING:

            # UPDATE UNLOADING STATUS HERE

            self.current_status = RobotStatusEnum.DONE_UNLOADING
            self.get_logger().info(f"unloading: {self.current_status}")
        # Done
        elif self.current_status == RobotStatusEnum.DONE_UNLOADING:
            self.current_status = RobotStatusEnum.JOB_DONE
            self.current_load_status = 0.0
            self.sub_task_undocking()
            self.get_logger().info(f"unloading: {self.current_status}")
    # endregion
    # endregion

    # region ROBOT STATUS
    def set_battery_status(self, robot_status: RobotStatus):
        """
        Updates the battery status of the given RobotStatus object.
        Sets the battery level and calculates the estimated operation time remaining
        after charging based on the current battery capacity, percentage, and current draw.
        If capacity or current is zero, sets a default message indicating no remaining time.
        Args:
            robot_status (RobotStatus): The RobotStatus object to update with battery information.
        Returns:
            None
        """

        robot_status.battery_level = self.battery_status.percentage

        if self.battery_status.capacity > 0.0 and self.battery_status.current > 0.0:
            # time remaining to completely discharge the battery
            time_remaining = (self.battery_status.capacity *
                              self.battery_status.percentage/self.battery_status.current)
            # converting time to string
            robot_status.operation_hours_after_charging = self.convert_to_hhmmss(
                time_remaining)
        else:
            robot_status.operation_hours_after_charging = "00 hours 00 minutes remaining approx..."

    def set_estop_status(self, robot_status: RobotStatus):
        """
        Updates the online status flag of the given RobotStatus object based on the current estop status.
        If the emergency stop (estop) status is active (i.e., self.estop_status.data is truthy),
        sets the robot's online_flag to the estop status value. Otherwise, sets the online_flag
        to the default ONLINE value from OnlineFlagEnum.
        Args:
            robot_status (RobotStatus): The RobotStatus object whose online_flag will be updated.
        """

        if self.estop_status.data:
            robot_status.online_flag = self.estop_status.data
        else:
            robot_status.online_flag = OnlineFlagEnum.ONLINE.value

    def set_location_status(self, robot_status: RobotStatus):

        if self.pose_status:
            robot_status.topo_map_position = self.pose_status.position
            robot_status.topo_map_orientation = self.pose_status.orientation

        # now = Time()
        # transform: TransformStamped = self.tf2_buffer.lookup_transform(
        #     'map',        # target frame
        #     'base_link',  # source frame
        #     now,
        #     timeout=Duration(seconds=1)
        # )

        # Extract translation (robot position)
        # robot_status.topo_map_position.x = transform.transform.translation.x
        # robot_status.topo_map_position.y = transform.transform.translation.y
        # robot_status.topo_map_position.z = transform.transform.translation.z

        # robot_status.topo_map_orientation.x = transform.transform.rotation.x
        # robot_status.topo_map_orientation.y = transform.transform.rotation.y
        # robot_status.topo_map_orientation.z = transform.transform.rotation.z
        # robot_status.topo_map_orientation.w = transform.transform.rotation.w

        # if self.odom_status is not None:
        #     # Uncomment this section when using on actual robot
        #     # robot_status.gps_location.x = self.odom_status.pose.pose.position.x or 0
        #     # robot_status.gps_location.y = self.odom_status.pose.pose.position.y or 0
        #     # robot_status.gps_location.z = self.odom_status.pose.pose.position.z or 0

        #     # # Actual position of robot in XY based on the GPS position mapping to local topo map
        #     # x,y,z = self.lat_lon_to_cartesian(
        #     #     self.gps_status.latitude,
        #     #     self.gps_status.longitude,

        #     # # Robot Pose
        #     # robot_status.topo_map_position = self.pose_status.position

        #     # # IMU Data
        #     # robot_status.topo_map_orientation = self.imu_status.orientation

        #     # For simulation purpose only. Do not use below code when running actual robot
        #     # Robot Pose
        #     robot_status.topo_map_position = self.odom_status.pose.pose.position

        #     # IMU Data
        #     robot_status.topo_map_orientation = self.odom_status.pose.pose.orientation
    # endregion

    # region HELPER DEFINITIONS
    def scale_value(self,
                    value: float,
                    original_min: float,
                    original_max: float,
                    target_min: float,
                    target_max: float) -> float:
        """
        Scales a value from one range to another.

        Args:
            value (float): The value to scale.
            original_min (float): The minimum value of the original range.
            original_max (float): The maximum value of the original range.
            target_min (float): The minimum value of the target range.
            target_max (float): The maximum value of the target range.

        Returns:
            float: The scaled value in the target range.

        Raises:
            ValueError: If the original range is zero.
        """
        # Ensure the original range is not zero to avoid division by zero
        if original_max == original_min:
            raise ValueError("Original range cannot be zero.")

        # Apply the linear transformation formula to scale the values
        diff_value: float = value - original_min
        original_range: float = original_max - original_min
        target_range: float = target_max - target_min

        scaled_value: float = ((diff_value / original_range)
                               * target_range + target_min)

        return scaled_value

    def convert_to_hhmmss(self, hours: float) -> str:
        """
        Converts a duration in hours to a string in HH:MM:SS format.

        Args:
            hours (float): Duration in hours.

        Returns:
            str: Duration in HH:MM:SS format.
        """
        seconds = int(hours * 3600)  # Convert hours to seconds
        minutes, seconds = divmod(seconds, 60)  # Convert minutes
        hours, minutes = divmod(minutes, 60)  # Convert hours
        return f"{hours:02} hours {minutes:02} minutes remaining approximately."

    def lat_lon_to_cartesian(self,
                             lat: float,
                             lon: float,
                             alt: float) -> tuple[float, float, float]:
        """
        Converts latitude, longitude, and altitude to Cartesian coordinates.

        Args:
            lat (float): Latitude in degrees.
            lon (float): Longitude in degrees.
            alt (float): Altitude in meters.

        Returns:
            tuple: Cartesian coordinates (x, y, z) in meters.
        """
        # Define the WGS84 CRS (latitude, longitude, altitude)
        wgs84 = pyproj.CRS("EPSG:4326")  # WGS84 Latitude/Longitude (LLA)

        # Update this CRS to match the required CRS
        # Define the WGS84 geocentric CRS (x, y, z)
        geocentric = pyproj.CRS("EPSG:4978")  # WGS84 3D Cartesian (x, y, z)

        # Create a transformer from WGS84 (LLA) to WGS84 geocentric (x, y, z)
        transformer = pyproj.Transformer.from_crs(
            wgs84, geocentric, always_xy=True)

        x: float
        y: float
        z: float
        # Transform the latitude, longitude, and altitude to Cartesian coordinates (x, y, z)
        # Note the order: lon, lat, alt
        x, y, z = transformer.transform(lon, lat, alt)

        return x, y, z
    # endregion

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
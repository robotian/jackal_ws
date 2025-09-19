from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import SetRemap, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # imu_qos_profile = QoSProfile(
    #     reliability=QoSReliabilityPolicy.RELIABLE, # Or RELIABLE, but BEST_EFFORT is often better for high freq sensor data
    #     history=QoSHistoryPolicy.KEEP_LAST,
    #     depth=5 # Adjust based on how many messages you want to buffer
    # )

    project_bringup = get_package_share_directory("project_bringup")
    namespace = LaunchConfiguration('namespace')    
    parameters = LaunchConfiguration('parameters')
    robot_namespace = LaunchConfiguration('robot_namespace')

    arg_robot_namespace = DeclareLaunchArgument(
        'robot_namespace',
        default_value='/j100_0921')


    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='')

    arg_parameters = DeclareLaunchArgument(
        'parameters',
        default_value=PathJoinSubstitution([
          FindPackageShare('project_bringup'),
          'config',
          'imu_filter.yaml'
          ]))


    remappings=[
        ('imu/data_raw','data'),
        # ('imu/mag','sensors/imu_0/magnetic_field'),
        ('imu/data','data_filtered'),
        ('/tf','/j100_0921/tf'),
    ]

    launch_filter = Node(
                package='imu_filter_madgwick',
                executable='imu_filter_madgwick_node',
                name='madgwick_filter',
                namespace=namespace,
                output='screen',
                parameters=[parameters],
                # remappings=remappings,
                # Add QoS settings for the subscriber to the raw IMU data
                # qos_overrides={
                #     'data': imu_qos_profile,
                #     'imu/mag': imu_qos_profile,
                # }
            )

    return LaunchDescription(
        [
            arg_robot_namespace,
            arg_namespace,
            arg_parameters,
            launch_filter,            
        ]
    )
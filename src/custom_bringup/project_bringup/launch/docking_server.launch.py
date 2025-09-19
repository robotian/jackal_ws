import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('project_bringup')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='j100_0921',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    docking_params_file = os.path.join(
        bringup_dir, 'config','j100', 'docking_config.yaml')

    docking_server = Node(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        output='screen',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time},
                    docking_params_file],
        remappings=remappings)

    # Load the lifecycle manager for the docking server
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_docking',
        output='screen',
        namespace=namespace,
        parameters=[{'autostart': True}, {'node_names': ['docking_server']}],
    )

    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        # SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        docking_server,
        lifecycle_manager
    ])

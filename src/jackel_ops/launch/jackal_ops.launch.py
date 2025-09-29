from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare namespace argument
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='j100_0921',
        description='Namespace for the robot'
    )

    namespace = LaunchConfiguration('namespace')

    # Odom bringup
    odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('project_bringup'),
                'launch',
                'indoor_ZED_mocap_project_bup.launch.py'
            )
        ])
    )

    # Nav2 bringup
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('project_bringup'),
                'launch',
                'indoor_nav2_bringup.launch.py'
            )
        ]),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': 'true'
        }.items()
    )

    # Jackal Ops system_status
    jackal_ops = Node(
        package='jackel_ops',
        executable='system_status',
        name='system_status',
        namespace=namespace,
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        output='screen'
    )

    return LaunchDescription([
        namespace_arg,
        odom_launch,
        nav2_launch,
        jackal_ops
    ])
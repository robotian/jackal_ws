from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    project_bringup = get_package_share_directory("project_bringup")
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    # gps_namespace = LaunchConfiguration('gps_namespace')
    logger = launch.substitutions.LaunchConfiguration("log_level")

    ekf_parameter = LaunchConfiguration('ekf_parameter') 
    # navsat_trans_parameter = LaunchConfiguration('navsat_trans_parameter') 

    arg_ekf_parameter = DeclareLaunchArgument(
        'ekf_parameter',
        default_value=PathJoinSubstitution([
          FindPackageShare('project_bringup'),
          'config',
        #   'dualEKF_ekf_config.yaml'
          'dualEKF_ekf_global.yaml',
        ]))

    return LaunchDescription([
        arg_ekf_parameter,
        # arg_navsat_trans_parameter,
        launch.actions.DeclareLaunchArgument(
            "log_level",
            default_value=["info"],
            description="Logging level",
            ),        
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global_node',
            output='screen',
            namespace=namespace,
            parameters=[ekf_parameter],
            remappings=[
                ('/tf','tf'),
                ('/tf_static','tf_static'),
                ('odometry/filtered','ekf_global_node/odometry/filtered'),
                ('set_pose','ekf_global_node/set_pose'),
                 ],
            arguments=['--ros-args', '--log-level', logger]
           ),
        # launch_ros.actions.Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_local_node',
        #     output='screen',
        #     namespace=namespace,
        #     parameters=[ekf_parameter],
        #     remappings=[
        #         ('/tf','tf'),
        #         ('/tf_static','tf_static'),
        #         ('odometry/filtered','ekf_local_node/odometry/filtered'),
        #         ('set_pose','ekf_local_node/set_pose'),
        #          ],
        #     arguments=['--ros-args', '--log-level', logger]
        #    ),
        launch_ros.actions.Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node_front',
            output='screen',
            namespace=namespace,
            remappings=[('/tf','tf'),
                        ('/tf_static','tf_static'),
                        ('gps/fix','sensors/gps_2/fix'),
                        ('odometry/gps','navsat_node/odometry/gps_front'),  # this will be fed to EKF global
                        ('odometry/filtered','ekf_global_node/odometry/filtered'),
                        # ('imu/data','imu'),
                        ],
            parameters=[ekf_parameter],
           ),
])

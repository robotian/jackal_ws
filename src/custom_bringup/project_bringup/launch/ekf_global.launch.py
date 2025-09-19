# Copyright 2018 Open Source Robotics Foundation, Inc.
# Copyright 2019 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
          'ekf_global.yaml'
        ]))
    # arg_navsat_trans_parameter = DeclareLaunchArgument(
    #     'navsat_trans_parameter',
    #     default_value=PathJoinSubstitution([
    #       FindPackageShare('project_bringup'),
    #       'config',
    #       'navsat_transform.yaml'
    #     ]))


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
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            namespace=namespace,
            # remappings=[('/tf','tf'),
            #             ('/tf_static','tf_static'),
            #             ('gps/fix','navfix'),
            #             ('imu/data','imu')],
            # parameters=[os.path.join(project_bringup, 'config', 'navsat_transform.yaml')],
            parameters=[ekf_parameter],
           ),
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global_node',
            output='screen',
            namespace=namespace,
            # parameters=[os.path.join(project_bringup, 'config', 'ekf_global.yaml'),
            #             {'use_sim_time': use_sim_time }],
            parameters=[ekf_parameter],
            remappings=[('/tf','tf'),('/tf_static','tf_static') ],
            arguments=['--ros-args', '--log-level', logger]
           ),
])

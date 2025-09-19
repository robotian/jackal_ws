# Software License Agreement (BSD)
#
# @author    Luis Camero <lcamero@clearpathrobotics.com>
# @copyright (c) 2024, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch.substitutions import (
    LaunchConfiguration,
    Command,
    TextSubstitution,
    PathJoinSubstitution
)

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


# Enable colored output
os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

HIDDEN = [
    # 'odom',
    'path_map',
    'path_odom',
    'atm_press',
    'left_gray/camera_info',
    'left_gray/image_rect_gray',
    'left_gray/image_rect_gray/compressed',
    'left_gray/image_rect_gray/compressedDepth',
    'left_gray/image_rect_gray/ffmpeg',
    'left_gray/image_rect_gray/theora',
    'left_raw/camera_info',
    'left_raw/image_raw_color',
    'left_raw/image_raw_color/compressed',
    'left_raw/image_raw_color/compressedDepth',
    'left_raw/image_raw_color/ffmpeg',
    'left_raw/image_raw_color/theora',
    'left_raw_gray/camera_info',
    'left_raw_gray/image_raw_gray',
    'left_raw_gray/image_raw_gray/compressed',
    'left_raw_gray/image_raw_gray/compressedDepth',
    'left_raw_gray/image_raw_gray/ffmpeg',
    'left_raw_gray/image_raw_gray/theora',
    'right_gray/camera_info',
    'right_gray/image_rect_gray',
    'right_gray/image_rect_gray/compressed',
    'right_gray/image_rect_gray/compressedDepth',
    'right_gray/image_rect_gray/ffmpeg',
    'right_gray/image_rect_gray/theora',
    'right_raw/camera_info',
    'right_raw/image_raw_color',
    'right_raw/image_raw_color/compressed',
    'right_raw/image_raw_color/compressedDepth',
    'right_raw/image_raw_color/ffmpeg',
    'right_raw/image_raw_color/theora',
    'right_raw_gray/camera_info',
    'right_raw_gray/image_raw_gray',
    'right_raw_gray/image_raw_gray/compressed',
    'right_raw_gray/image_raw_gray/compressedDepth',
    'right_raw_gray/image_raw_gray/ffmpeg',
    'right_raw_gray/image_raw_gray/theora',
# /j100_0921/sensors/camera_0/stereolabs_zed/left_raw/image_raw_color/compressed
# /j100_0921/sensors/camera_0/stereolabs_zed/left_raw/image_raw_color/compressedDepth
# /j100_0921/sensors/camera_0/stereolabs_zed/left_raw/image_raw_color/ffmpeg
# /j100_0921/sensors/camera_0/stereolabs_zed/left_raw/image_raw_color/theora
# /j100_0921/sensors/camera_0/stereolabs_zed/left_raw_gray/image_raw_gray/compressed
# /j100_0921/sensors/camera_0/stereolabs_zed/left_raw_gray/image_raw_gray/compressedDepth
# /j100_0921/sensors/camera_0/stereolabs_zed/left_raw_gray/image_raw_gray/ffmpeg
# /j100_0921/sensors/camera_0/stereolabs_zed/left_raw_gray/image_raw_gray/theora


    
    # 'pose',
    # 'pose/status',
    # 'pose_with_covariance',
    # 'left_cam_imu_transform',
    # ''
]

CAMERAS = [
    # Depth
    ('depth', 'depth'),
    # Left
    ('left', 'left'),
    ('left_gray', 'left/gray'),
    ('left_raw', 'left/raw'),
    ('left_raw_gray', 'left/raw_gray'),
    # Color
    ('rgb', 'color'),
    ('rgb_gray', 'color/gray'),
    ('rgb_raw', 'color/raw'),
    ('rgb_raw_gray', 'color/raw_gray'),
    # Right
    ('right', 'right'),
    ('right_gray', 'right/gray'),
    ('right_raw', 'right/raw'),
    ('right_raw_gray', 'right/raw_gray'),
    # Stereo
    ('stereo', 'stereo'),
    ('stereo_raw', 'stereo/raw')
]

OTHERS = [
    'temperature/imu',
    'temperature/left',
    'temperature/right',
    'imu/data',
    'imu/data_raw',
    'imu/mag',
    'atm_press',
    'confidence/confidence_map',
    'disparity/disparity_image',
    'point_cloud/cloud_registered',
]

# Depth Image to laser scan config file
default_config_cvt = os.path.join(
    get_package_share_directory('project_bringup'),
    'config','zed',
    'zed_depth_to_laserscan.yaml'
)


def generate_launch_description():
    parameters = LaunchConfiguration('parameters')
    namespace = LaunchConfiguration('namespace')
    robot_namespace = LaunchConfiguration('robot_namespace')

    config_path_cvt = LaunchConfiguration('config_path_cvt')

    arg_parameters = DeclareLaunchArgument(
        'parameters',
        default_value=PathJoinSubstitution([
          FindPackageShare('project_bringup'),
          'config',
          'zed_common.yaml'
        ]))

    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='')

    arg_robot_namespace = DeclareLaunchArgument(
        'robot_namespace',
        default_value='')

    arg_config_path_cvt = DeclareLaunchArgument(
                'config_path_cvt',
                default_value=TextSubstitution(text=default_config_cvt),
                description='Path to the YAML configuration file for the detpth to laser scan conversion.')

    remappings = []


    # Hidden Topics
    for hidden in HIDDEN:
        remappings.append(
            ('~/%s' % hidden, PathJoinSubstitution(['/', namespace, '_' + hidden]))
        )

    # Cameras
    # for old, new in CAMERAS:
    #     if 'depth' in old:
    #         image = 'depth_registered'
    #         remappings.append(
    #             ('~/%s/depth_info' % (old),
    #                 PathJoinSubstitution(['/', namespace, new, 'depth_info'])),
    #         )
    #     elif 'raw_gray' in old:
    #         image = 'image_raw_gray'
    #     elif 'raw' in old:
    #         image = 'image_raw_color'
    #     elif 'gray' in old:
    #         image = 'image_rect_gray'
    #     else:
    #         image = 'image_rect_color'
    #     remappings.extend([
    #         ('~/%s/camera_info' % (old),
    #             PathJoinSubstitution(['/', namespace, new, 'camera_info'])),
    #         ('~/%s/%s' % (old, image),
    #             PathJoinSubstitution(['/', namespace, new, 'image'])),
    #         ('~/%s/%s/compressed' % (old, image),
    #             PathJoinSubstitution(['/', namespace, new, 'compressed'])),
    #         ('~/%s/%s/compressedDepth' % (old, image),
    #             PathJoinSubstitution(['/', namespace, new, 'compressedDepth'])),
    #         ('~/%s/%s/theora' % (old, image),
    #             PathJoinSubstitution(['/', namespace, new, 'theora'])),
    #     ])

    # Others
    # for topic in OTHERS:
    #     remappings.append(
    #         ('~/%s' % topic,
    #             PathJoinSubstitution(['/', namespace, topic])),
    #     )

    # Transforms
    remappings.append(('/tf', PathJoinSubstitution(['/', robot_namespace, 'tf'])))
    remappings.append(('/tf_static', PathJoinSubstitution(['/', robot_namespace, 'tf_static'])))

    stereolabs_zed_node = ComposableNode(
        package='zed_components',
        namespace=namespace,
        plugin='stereolabs::ZedCamera',
        name='stereolabs_zed',
        parameters=[parameters],
        remappings=remappings,
        extra_arguments=[{'use_intra_process_comms': True}],  # Enable intra-process communication
    )

    # Depth Image to Laser Scan component
    # This component converts the depth image from the ZED camera to a laser scan format.
    # It uses the depthimage_to_laserscan package to perform the conversion.
    # The configuration file for this component is specified by the config_path_cvt argument.
    zed_cvt_component = ComposableNode(
        package='depthimage_to_laserscan',
        namespace=robot_namespace,
        plugin='depthimage_to_laserscan::DepthImageToLaserScanROS',
        name='depthimage_to_laserscan',
        parameters=[
            config_path_cvt,
            {'output_frame': 'camera_0_left_camera_frame'},
            ],
        remappings=[
                ('depth',  'sensors/camera_0/depth/image'),  # (Todo) this should use the namespace
                ('depth_camera_info',  'sensors/camera_0/depth/camera_info') # (Todo) this should use the namespace
            ],
    )

    bringup_dir = get_package_share_directory('project_bringup')


    declare_apriltag_config_cmd = DeclareLaunchArgument(
        'apriltag_config',
        default_value=os.path.join(bringup_dir, 'config', 'j100','tags_36h11.yaml'),
        description='Path to the apriltag configuration file')

    apriltag_component = ComposableNode(
        package='apriltag_ros',
        namespace=namespace,
        plugin='apriltag_ros::AprilTagNode',
        name='apriltag_node',
        # parameters=[{
        #     'family': 'tag36h11',
        #     'size': 0.162,  # Size of the tag in meters
        #     'max_hamming': 0,
        #     'tag_ids': [],  # Empty list means detect all tag IDs
        #     'camera_frame': 'camera_0_left_camera_frame',
        #     'publish_tf': True,
        #     'tf_prefix': '',
        #     'camera_info_topic': 'left/camera_info',
        #     'image_topic': 'left/image_rect_color',
        #     'queue_size': 10,
        # }],
        parameters=[LaunchConfiguration('apriltag_config')],
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('image_rect', 'sensors/camera_0/stereolabs_zed/rgb/image_rect_color'),
                    ('camera_info', 'sensors/camera_0/stereolabs_zed/rgb/camera_info'),
                    ('detections', 'apriltag_detections'),
                    ],
        extra_arguments=[{'use_intra_process_comms': True}],  # Enable intra-process communication
    )   

    image_processing_container = ComposableNodeContainer(
        name='image_processing_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
                                    stereolabs_zed_node, 
                                    # apriltag_component,
                                    # zed_cvt_component,
                                    ],
        output='screen',
        remappings=remappings,
        arguments=['--ros-args', '--log-level', 'info'],
    )


    ld = LaunchDescription()
    ld.add_action(arg_parameters)
    ld.add_action(arg_namespace)
    ld.add_action(arg_robot_namespace)
    # ld.add_action(arg_config_path_cvt)
    # ld.add_action(zed_cvt_component) 
    # ld.add_action(declare_apriltag_config_cmd)
    ld.add_action(image_processing_container) 
    
    return ld

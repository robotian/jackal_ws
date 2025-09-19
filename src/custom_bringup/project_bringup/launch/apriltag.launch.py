
# ros2 run apriltag_ros apriltag_node --ros-args     -r image_rect:=zed_node/left/image_rect_color     -r camera_info:=zed_node/left/camera_info     --params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml -r __ns:=/husky1 -r /tf:=tf

# launch file for apriltag detection
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument    


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('project_bringup')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_apriltag_config_cmd = DeclareLaunchArgument(
        'apriltag_config',
        default_value=os.path.join(bringup_dir, 'config', 'j100','tags_36h11.yaml'),
        description='Path to the apriltag configuration file')
    
    # run the apriltag node
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    LaunchConfiguration('apriltag_config')],
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('image_rect', 'sensors/camera_0/stereolabs_zed/rgb/image_rect_color'),
                    ('camera_info', 'sensors/camera_0/stereolabs_zed/rgb/camera_info'),
                    ('detections', 'apriltag_detections'),
                    ],
    )

    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_sim_time_cmd,  
        declare_apriltag_config_cmd,              
        apriltag_node,        
    ])

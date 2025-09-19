from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap, PushRosNamespace

def generate_launch_description():
    # Package Directory
    # pkg_project_bringup = FindPackageShare('project_bringup')
    robot_namespace = LaunchConfiguration('robot_namespace')
    arg_robot_namespace = DeclareLaunchArgument(
        'robot_namespace',
        default_value='/j100_0921')

    rtab_launch_file = PathJoinSubstitution([FindPackageShare('rtabmap_launch'), 'launch', 'rtabmap.launch.py'])

    # Include Launch
    launch_rtabmap = GroupAction(
        actions=[            
            # PushRosNamespace(LaunchConfiguration('robot_namespace')),
            SetRemap(src='/tf',dst='tf'),
            SetRemap(src='/tf_static',dst='tf_static'),
            # SetRemap(src='/tf_static',dst=[LaunchConfiguration('robot_namespace'),'/tf_static']),
            # SetRemap(src='navsatfix',dst='fix'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([rtab_launch_file]),  
                launch_arguments={
                    ('namespace', LaunchConfiguration('robot_namespace')),
                    ('rtabmap_args','--delete_db_on_start'),
                    ('stereo','true'),
                    ('odom_topic','/j100_0921/odometry/filtered'),
                    ('frame_id', 'base_link'),
                    ('map_frame_id', 'map_rtabmap'),
                    ('wait_imu_to_init', 'false'),
                    ('visual_odometry', 'false'),
                    ('stereo_namespace','/j100_0921/sensors/camera_0/'),
                    ('left_image_topic', '/j100_0921/sensors/camera_0/left/image'),
                    ('right_image_topic', '/j100_0921/sensors/camera_0/right/image'),
                    ('left_camera_info_topic', '/j100_0921/sensors/camera_0/left/camera_info'),
                    ('right_camera_info_topic', '/j100_0921/sensors/camera_0/right/camera_info'),  
                    ('wait_for_transform','1.0'),  
                    ('rtabmap_viz', 'false'),     
                    ('approx_sync,','true'),           
                    # ('base_frame_id', 'base_link'),
                    # ('odom_frame_id', 'odom'),
                    # ('robot_namespace', LaunchConfiguration('robot_namespace')),
                },
            )
        ]
    )

    ld = LaunchDescription()
    ld.add_action(arg_robot_namespace)
    ld.add_action(launch_rtabmap)
    return ld


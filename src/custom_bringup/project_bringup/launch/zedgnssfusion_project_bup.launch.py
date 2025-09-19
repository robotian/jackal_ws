import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetRemap, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_namespace = LaunchConfiguration('robot_namespace')
    arg_robot_namespace = DeclareLaunchArgument(
        'robot_namespace',
        default_value='/j100_0921')

    zed_namespace = LaunchConfiguration('zed_namespace')
    arg_zed_namespace = DeclareLaunchArgument(
        'zed_namespace',
        default_value=[LaunchConfiguration('robot_namespace'),'/sensors/camera_0'])
    zed_param = LaunchConfiguration('zed_param') 
    arg_zed_param = DeclareLaunchArgument(
        'zed_param',
        default_value=PathJoinSubstitution([
          FindPackageShare('project_bringup'),
          'config',
          'zedgnssfusion_zed_common_v2.yaml'
        ]))

    # front duro
    duro_namespace = LaunchConfiguration('duro_namespace')
    arg_duro_namespace = DeclareLaunchArgument(
        'duro_namespace',
        default_value=[LaunchConfiguration('robot_namespace'),'/sensors/gps_1'])

    duro_parameter =  LaunchConfiguration('duro_parameter')
    arg_duro_parameters = DeclareLaunchArgument(
        'duro_parameter',
        default_value=PathJoinSubstitution([
          FindPackageShare('project_bringup'),
          'config',
          'swiftnav_duro_front.yaml'
        ]))

    # if you have a second Duro
    duro_rear_namespace = LaunchConfiguration('duro_rear_namespace')
    arg_duro_rear_namespace = DeclareLaunchArgument(
        'duro_rear_namespace',
        default_value=[LaunchConfiguration('robot_namespace'),'/sensors/gps_2'])

    duro_rear_parameter =  LaunchConfiguration('duro_rear_parameter')
    arg_duro_rear_parameters = DeclareLaunchArgument(
        'duro_rear_parameter',
        default_value=PathJoinSubstitution([
          FindPackageShare('project_bringup'),
          'config',
          'swiftnav_duro_rear.yaml'
        ]))


    microstrain_namespace = LaunchConfiguration('microstrain_namespace') 
    arg_microstrain_namespace = DeclareLaunchArgument(
        'microstrain_namespace',
        default_value=[LaunchConfiguration('robot_namespace'),'/sensors/imu_1'])

    microstrain_param = LaunchConfiguration('microstrain_param') 
    arg_microstrain_param = DeclareLaunchArgument(
        'microstrain_param',
        default_value=PathJoinSubstitution([
          FindPackageShare('project_bringup'),
          'config',
          'microstrain_imu.yaml'
        ]))
   
    # Include Packages
    pkg_clearpath_sensors = FindPackageShare('clearpath_sensors')
    

    # Package Directory
    pkg_project_bringup = FindPackageShare('project_bringup')

    # Launch File
    duro_launch = PathJoinSubstitution([pkg_project_bringup, 'launch', 'swiftnav_duro.launch.py'])

    # Include Launch
    launch_duro = GroupAction(
        actions=[            
            PushRosNamespace(LaunchConfiguration('duro_namespace')),
            SetRemap(src='/tf',dst=[LaunchConfiguration('robot_namespace'),'/tf']),
            SetRemap(src='/tf_static',dst=[LaunchConfiguration('robot_namespace'),'/tf_static']),
            SetRemap(src='navsatfix',dst='fix'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([duro_launch]),  
                launch_arguments={(
                    'parameters'
                    ,
                    duro_parameter
                )},
            )
        ]
    )

    # Launch File
    duro_rear_launch = PathJoinSubstitution([pkg_project_bringup, 'launch', 'swiftnav_duro.launch.py'])

    # Include Launch
    launch_duro_rear = GroupAction(
        actions=[            
            PushRosNamespace(LaunchConfiguration('duro_rear_namespace')),
            SetRemap(src='/tf',dst=[LaunchConfiguration('robot_namespace'),'/tf']),
            SetRemap(src='/tf_static',dst=[LaunchConfiguration('robot_namespace'),'/tf_static']),
            SetRemap(src='navsatfix',dst='fix'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([duro_launch]),  
                launch_arguments={(
                    'parameters'
                    ,
                    duro_rear_parameter
                )},
            )
        ]
    )

    zed_launch = PathJoinSubstitution([pkg_project_bringup, 'launch', 'stereolabs_zed.launch.py'])
    launch_zed = GroupAction(
        actions=[            
            # PushRosNamespace(LaunchConfiguration('zed_namespace')),
            SetRemap(src='/tf',dst=[LaunchConfiguration('robot_namespace'),'/tf']),
            SetRemap(src='/tf_static',dst=[LaunchConfiguration('robot_namespace'),'/tf_static']),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([zed_launch]),  
                launch_arguments={
                    ('namespace', zed_namespace),
                    ('parameters', zed_param),                    
                    },
            )
        ]
    )


    

    # Declare launch files
    launch_file_microstrain_imu = PathJoinSubstitution([
        pkg_clearpath_sensors, 'launch', 'microstrain_imu.launch.py'])

    # Include launch files
    launch_microstrain_imu = GroupAction(
        actions=[
            SetRemap(src='/tf',dst=[LaunchConfiguration('robot_namespace'),'/tf']),
            SetRemap(src='/tf_static',dst=[LaunchConfiguration('robot_namespace'),'/tf_static']),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([launch_file_microstrain_imu]),
                launch_arguments=
                    [
                        (
                            'parameters'
                            ,
                            microstrain_param
                        )
                        ,
                        (
                            'namespace'
                            ,
                            microstrain_namespace
                        )
                        ,
                        (
                            'robot_namespace'
                            ,
                            robot_namespace
                        )
                        ,
                    ]
            )
        ]
    )

    launch_file_ekf_global = PathJoinSubstitution([
        pkg_project_bringup, 'launch', 'ekf_global.launch.py'])

        # remappings=[('/tf','tf'),
        #                 ('/tf_static','tf_static'),
        #                 ('gps/fix','navfix'),
        #                 ('imu/data','imu')],
    launch_ekf_global = GroupAction(
        actions=[            
            PushRosNamespace(LaunchConfiguration('robot_namespace')),
            SetRemap(src='/tf',dst=[LaunchConfiguration('robot_namespace'),'/tf']),
            SetRemap(src='gps/fix',dst=[LaunchConfiguration('duro_namespace'),'/fix']),
            SetRemap(src='imu/data',dst=[LaunchConfiguration('duro_namespace'),'/imu']),
            # SetRemap(src='navsatfix',dst='fix'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([launch_file_ekf_global]),  
                launch_arguments={'use_sim_time': 'false',      
                          'namespace': robot_namespace,   
                        #   'gps_namespace': duro_namespace,                    
                          }.items()  # Optional: Pass arguments if needed                
            )
        ]
    )


    stf_camera2base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cam2base_stf',
        output='screen',
        namespace=robot_namespace,
        emulate_tty=True,
        arguments=['-0.252', '0', '-0.226', '0', '0', '0', 'camera_0_camera_link' , 'base_link'],  # check the fake_gps.launch.py for the correct frame names
        remappings=[('/tf','tf'),('/tf_static','tf_static') ],
    )


    launch_navsat_tf = Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            namespace=robot_namespace,
            emulate_tty=True,
            remappings=[
                ('/tf','tf'),
                ('/tf_static','tf_static'),
                ('gps/fix','sensors/gps_1/fix'),
                ('odometry/filtered','sensors/camera_0/stereolabs_zed/odom'),
                ],
            parameters=[os.path.join(get_package_share_directory("project_bringup"), 'config', 'navsat_transform.yaml')],
    )


    # Launch Description
    ld = LaunchDescription()
    ld.add_action(arg_robot_namespace)
    ld.add_action(arg_duro_namespace) 
    ld.add_action(arg_duro_rear_namespace)   
    ld.add_action(arg_zed_namespace)   
    ld.add_action(arg_microstrain_namespace) 
    ld.add_action(arg_duro_parameters)   
    ld.add_action(arg_duro_rear_parameters)             
    ld.add_action(arg_microstrain_param)   
    ld.add_action(arg_zed_param)        
    ld.add_action(launch_duro) 
    ld.add_action(launch_duro_rear) 
    # ld.add_action(stf_camera2base) 
    ld.add_action(launch_zed) 
    # ld.add_action(launch_navsat_tf) 
    # ld.add_action(launch_microstrain_imu)
    # ld.add_action(launch_ekf_global)
    return ld
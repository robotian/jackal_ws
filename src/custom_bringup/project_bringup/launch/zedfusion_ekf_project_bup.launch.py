from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import SetRemap, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource


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
          'zedfusion_ekf_zed_common.yaml'
        ]))


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
          'swiftnav_duro.yaml'
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

    ekf_parameter = LaunchConfiguration('ekf_parameter') 
    # navsat_trans_parameter = LaunchConfiguration('navsat_trans_parameter') 

    arg_ekf_parameter = DeclareLaunchArgument(
        'ekf_parameter',
        default_value=PathJoinSubstitution([
          FindPackageShare('project_bringup'),
          'config',
          'zedfusion_ekf_global.yaml'
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

    imu_filter_launch_file = PathJoinSubstitution([pkg_project_bringup, 'launch', 'imu_filter.launch.py'])

    #     remappings=[
    #     ('imu/data_raw','data'),
    #     # ('imu/mag','sensors/imu_0/magnetic_field'),
    #     ('imu/data','data_filtered'),
    #     ('/tf','/j100_0921/tf'),
    # ]

    launch_imu_filter = GroupAction(
        actions=[            
            PushRosNamespace(LaunchConfiguration('microstrain_namespace')),
            SetRemap(src='/tf',dst=[LaunchConfiguration('robot_namespace'),'/tf']),
            SetRemap(src='imu/data_raw',dst=[LaunchConfiguration('microstrain_namespace'),'/data']),
            SetRemap(src='imu/mag',dst=[LaunchConfiguration('microstrain_namespace'),'/imu/mag']),
            SetRemap(src='imu/data',dst=[LaunchConfiguration('microstrain_namespace'),'/data_filtered']),
            # SetRemap(src='navsatfix',dst='fix'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([imu_filter_launch_file]),  
                launch_arguments={
                        #   'use_sim_time': 'false',      
                          'namespace': microstrain_namespace,   
                        #   'parameters': ekf_parameter,
                        #   'gps_namespace': duro_namespace,                    
                          }.items()  # Optional: Pass arguments if needed                
            )
        ]
    )

    launch_file_ekf_local = PathJoinSubstitution([
        pkg_project_bringup, 'launch', 'ekf_local.launch.py'])
    launch_ekf_local = GroupAction(
        actions=[            
            PushRosNamespace(LaunchConfiguration('robot_namespace')),
            SetRemap(src='odometry/filtered',dst='odometry/filtered_local'),
            # SetRemap(src='/tf',dst=[LaunchConfiguration('robot_namespace'),'/tf']),
            # SetRemap(src='/tf_static',dst=[LaunchConfiguration('robot_namespace'),'/tf_static']),
            
            # SetRemap(src='navsatfix',dst='fix'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([launch_file_ekf_local]),  
                # launch_arguments={'use_sim_time': 'false',      
                #           'namespace': robot_namespace,   
                #         #   'parameters': ekf_parameter,
                #         #   'gps_namespace': duro_namespace,                    
                #           }.items()  # Optional: Pass arguments if needed                
            )
        ]
    )

    launch_file_ekf_global = PathJoinSubstitution([
        pkg_project_bringup, 'launch', 'ekf_global.launch.py'])

    launch_ekf_global = GroupAction(
        actions=[            
            PushRosNamespace(LaunchConfiguration('robot_namespace')),
            SetRemap(src='/tf',dst=[LaunchConfiguration('robot_namespace'),'/tf']),
            SetRemap(src='/tf_static',dst=[LaunchConfiguration('robot_namespace'),'/tf_static']),
            SetRemap(src='gps/fix',dst=[LaunchConfiguration('duro_namespace'),'/fix']),
            SetRemap(src='imu/data',dst=[LaunchConfiguration('duro_namespace'),'/imu']),
            # SetRemap(src='navsatfix',dst='fix'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([launch_file_ekf_global]),  
                launch_arguments={'use_sim_time': 'false',      
                          'namespace': robot_namespace,   
                          'parameters': ekf_parameter,
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


    # Launch Description
    ld = LaunchDescription()
    ld.add_action(arg_robot_namespace)
    ld.add_action(arg_duro_namespace)   
    ld.add_action(arg_zed_namespace)   
    ld.add_action(arg_ekf_parameter)
    ld.add_action(arg_microstrain_namespace) 
    ld.add_action(arg_duro_parameters)             
    ld.add_action(arg_microstrain_param)   
    ld.add_action(arg_zed_param)        
    ld.add_action(launch_duro) 
    # ld.add_action(stf_camera2base) 
    ld.add_action(launch_zed) 
    ld.add_action(launch_microstrain_imu)
    ld.add_action(launch_imu_filter)
    # ld.add_action(launch_ekf_global)
    ld.add_action(launch_ekf_local)
    return ld
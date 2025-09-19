# Dual GPS and Dual EKF approach
# ZED VIO for local odom
# 


import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetRemap, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    # Include Packages
    pkg_clearpath_sensors = FindPackageShare('clearpath_sensors')   
    pkg_swiftnav_duro = FindPackageShare('swiftnav_ros2_driver')
    pkg_project_bringup = FindPackageShare('project_bringup')

    robot_namespace = LaunchConfiguration('robot_namespace')
    zed_namespace = LaunchConfiguration('zed_namespace')
    attitude_duro_namespace = LaunchConfiguration('attitude_duro_namespace')
    reference_duro_namespace = LaunchConfiguration('reference_duro_namespace')


    zed_param = LaunchConfiguration('zed_param') 
    attitude_duro_parameter =  LaunchConfiguration('attitude_duro_parameter')
    reference_duro_parameter =  LaunchConfiguration('reference_duro_parameter')
    ekf_parameter = LaunchConfiguration('ekf_parameter') 

    # Declare the use_sim_time argument
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    arg_robot_namespace = DeclareLaunchArgument(
        'robot_namespace',
        default_value='/j100_0921')

    declare_use_ekfglobal_node_arg = DeclareLaunchArgument(
        'use_ekf_global',
        default_value='False',
        description='Set to True to launch EKF global and NavSat Transform nodes. You need to disable to use RTAB mapping'
    )

    arg_ekf_parameter = DeclareLaunchArgument(
        'ekf_parameter',
        default_value=PathJoinSubstitution([
          FindPackageShare('project_bringup'),
          'config',
        #   'dualEKF_ekf_global.yaml',
          'ZEDodom_EKF_ekf_global.yaml',
        ]))

    arg_zed_namespace = DeclareLaunchArgument(
        'zed_namespace',
        default_value=[LaunchConfiguration('robot_namespace'),'/sensors/camera_0'])

    arg_zed_param = DeclareLaunchArgument(
        'zed_param',
        default_value=PathJoinSubstitution([
          pkg_project_bringup,
          'config',
        #   'dualEKF_zed_common.yaml'
          'indoor_zed_common.yaml'
        ]))

    # attitude duro
    arg_attitude_duro_namespace = DeclareLaunchArgument(
        'attitude_duro_namespace',
        default_value=[LaunchConfiguration('robot_namespace'),'/sensors/gps_1'])

    arg_attitude_duro_parameter = DeclareLaunchArgument(
        'attitude_duro_parameter',
        default_value=PathJoinSubstitution([
          FindPackageShare('project_bringup'),
          'config',
          'j100',
          'duro_attitude.yaml'
        ]))


    # if you have a second Duro
    arg_reference_duro_namespace = DeclareLaunchArgument(
        'reference_duro_namespace',
        default_value=[LaunchConfiguration('robot_namespace'),'/sensors/gps_2'])

    arg_reference_duro_parameter = DeclareLaunchArgument(
        'reference_duro_parameter',
        default_value=PathJoinSubstitution([
          FindPackageShare('project_bringup'),
          'config',
          'j100',
          'duro_reference.yaml'
        ]))


    att_duro_node = Node(
        package='swiftnav_ros2_driver',
        executable='sbp-to-ros',
        name='att_duro_node',
        namespace=attitude_duro_namespace,
        remappings=[
            # ('/tf','/j100_0921/tf'),
            # ('/tf_static','/j100_0921/tf_static'),
            ('navsatfix','fix'),
        ],
        output='screen',
        parameters=[attitude_duro_parameter]
    )

    config = os.path.join(get_package_share_directory('project_bringup'), 'config', 'j100','imu_filter.yaml')

    # att_duro_imu_filter_node = Node(
    #     package='imu_complementary_filter',
    #     executable='complementary_filter_node',
    #     name='complementary_filter_att_node',
    #     output='screen',
    #     parameters=[config],
    #     namespace=robot_namespace,
    #     remappings=[
    #         ('/tf','tf'),
    #         ('/tf_static','tf_static'),
    #         ('imu/data_raw','sensors/gps_1/imu'),
    #         ('imu/data','sensors/gps_1/imu_filtered'),
    #     ],
    # )

    ref_duro_node = Node(
        package='swiftnav_ros2_driver',
        executable='sbp-to-ros',
        name='ref_duro_node',
        namespace=reference_duro_namespace,
        remappings=[
            # ('/tf','/j100_0921/tf'),
            # ('/tf_static','/j100_0921/tf_static'),
            ('navsatfix','fix'),
        ],
        output='screen',
        parameters=[reference_duro_parameter]
    )

    # ref_duro_imu_filter_node = Node(
    #     package='imu_complementary_filter',
    #     executable='complementary_filter_node',
    #     name='complementary_filter_ref_node',
    #     output='screen',
    #     parameters=[config],
    #     namespace=robot_namespace,
    #     remappings=[
    #         ('/tf','tf'),
    #         ('/tf_static','tf_static'),
    #         ('imu/data_raw','sensors/gps_2/imu'),
    #         ('imu/data','sensors/gps_2/imu_filtered'),
    #     ],
    # )


    ref_duro_imu_madg_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_madgwick_ref_node',
        output='screen',
        parameters=[config],
        namespace=robot_namespace,
        remappings=[
            ('/tf','tf'),
            ('/tf_static','tf_static'),
            ('imu/data_raw','sensors/gps_2/imu'),
            ('imu/data','sensors/gps_2/madgwick/imu_filtered'),
        ],
    )

    att_duro_imu_madg_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_madgwick_att_node',
        output='screen',
        parameters=[config],
        namespace=robot_namespace,
        remappings=[
            ('/tf','tf'),
            ('/tf_static','tf_static'),
            ('imu/data_raw','sensors/gps_1/imu'),
            ('imu/data','sensors/gps_1/madgwick/imu_filtered'),
        ],
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

    launch_file_ekf_global = PathJoinSubstitution([
        pkg_project_bringup, 'launch', 'ekf_navsat_global.launch.py'])

    launch_ekf_global = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_ekf_global')),
        actions=[            
            PushRosNamespace(LaunchConfiguration('robot_namespace')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([launch_file_ekf_global]),  
                launch_arguments={'use_sim_time': 'false',      
                          'namespace': robot_namespace,   
                          'ekf_parameter': ekf_parameter,
                        #   'gps_namespace': duro_namespace,                    
                          }.items()  # Optional: Pass arguments if needed                
            )
        ]
    )

    # launch_file_ekf_local = PathJoinSubstitution([
    #     pkg_project_bringup, 'launch', 'dualEKF_ekf_local.launch.py'])

    # launch_ekf_local = GroupAction(
    #     actions=[            
    #         PushRosNamespace(LaunchConfiguration('robot_namespace')),
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([launch_file_ekf_local]),  
    #             launch_arguments={'use_sim_time': 'false',      
    #                       'namespace': robot_namespace,   
    #                     #   'gps_namespace': duro_namespace,                    
    #                       }.items()  # Optional: Pass arguments if needed                
    #         )
    #     ]
    # )

    att_duro_headingOdom_node = Node(
        package='odom_transformer',
        executable='attitude_duro_heading2odom',
        name='attitude_duro_heading2odom',
        output='screen',
        namespace=robot_namespace,
        emulate_tty=True,
        remappings=[('/tf','tf'),('/tf_static','tf_static') ],
    )

    odom_publisher_node = Node(
        package='odom_transformer',  # Replace with your actual package name
        # executable='odom_transformer_node',  # The name of your Python script without the .py extension
        executable='odom_transformer_ekf_node',  # The name of your Python script without the .py extension
        name='zed_odom_transform',
        output='screen',
        namespace=robot_namespace,
        remappings=[('/tf','tf'),('/tf_static','tf_static') ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},     
                    {'target_odom_topic': 'sensors/camera_0/stereolabs_zed/odom'},  
                    {'publish_tf': True}  # Set to True to publish the transform    
                    ]
    )


    

    declare_apriltag_config_cmd = DeclareLaunchArgument(
        'apriltag_config',
        default_value = PathJoinSubstitution([pkg_project_bringup, 'config', 'j100','tags_36h11.yaml']),
        # default_value=os.path.join(pkg_project_bringup, 'config', 'j100','tags_36h11.yaml'),
        description='Path to the apriltag configuration file',
        )
    
    # run the apriltag node
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        namespace=robot_namespace,
        output='screen',
        parameters=[
            # {'use_sim_time': use_sim_time},
            LaunchConfiguration('apriltag_config'),
            ],
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('image_rect', 'sensors/camera_0/stereolabs_zed/rgb/image_rect_color'),
                    ('camera_info', 'sensors/camera_0/stereolabs_zed/rgb/camera_info'),
                    ('detections', 'apriltag_detections'),
                    ],
    )


    # Launch Description
    ld = LaunchDescription()
    ld.add_action(arg_use_sim_time)
    ld.add_action(arg_robot_namespace) 
    ld.add_action(arg_zed_namespace)   
    ld.add_action(arg_attitude_duro_namespace)   
    ld.add_action(arg_reference_duro_namespace)
    ld.add_action(arg_zed_param)    
    ld.add_action(arg_attitude_duro_parameter)
    ld.add_action(arg_reference_duro_parameter)
    ld.add_action(arg_ekf_parameter)    
    ld.add_action(declare_use_ekfglobal_node_arg)

    # ld.add_action(att_duro_node)
    # ld.add_action(ref_duro_node)


    # ld.add_action(ref_duro_imu_filter_node)
    # ld.add_action(ref_duro_imu_madg_node)   # madgwick filter
    # ld.add_action(att_duro_imu_madg_node)   # madgwick filter
    
    ld.add_action(launch_zed) 
    
    # ld.add_action(launch_ekf_local)
    ld.add_action(odom_publisher_node)

    # ld.add_action(att_duro_headingOdom_node)    

    ld.add_action(declare_apriltag_config_cmd)
    ld.add_action(apriltag_node)

    # # comment out for RTAB mapping
    ld.add_action(launch_ekf_global)
    
    
    
    return ld
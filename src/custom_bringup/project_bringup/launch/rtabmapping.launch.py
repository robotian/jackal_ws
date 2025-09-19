# Requirements:
#   A ZED camera
#   Install zed ros2 wrapper package (https://github.com/stereolabs/zed-ros2-wrapper)
# Example:
#   $ ros2 launch rtabmap_examples zed.launch.py camera_model:=zed2i

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition

import tempfile

parameters = []
remappings = []

def launch_setup(context: LaunchContext, *args, **kwargs):

    # Hack to override grab_resolution parameter without changing any files
    with tempfile.NamedTemporaryFile(mode='w+t', delete=False) as zed_override_file:
        zed_override_file.write("---\n"+
                  "/**:\n"+
                  "    ros__parameters:\n"+
                  "        general:\n"+
                  "            grab_resolution: 'VGA'")

    parameters=[{'frame_id':'base_link',
                 'subscribe_rgbd':True,
                 'approx_sync':True,  #If timestamps of the input topics should be synchronized using approximate or exact time policy.
                 'wait_imu_to_init':False,
                 'odom_frame_id': 'odom',
                 'approx_sync_max_interval': 0.01,
                 'tag_topic': 'apriltag_detections',
                 'tag_linear_variance': 0.001,
                 'tag_angular_variance': 9999.0,
                 'database_path': '~/.ros/rtabmap_lab1.db',
                #  'gps_topic': 'sensors/gps_2/fix',
                #  'landmark_linear_variance': 0.01,  # landmark linear variance in odom frame
                #  'landmark_angular_variance': float(9999),  # landmark angular variance
                #  'initial_pose': [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], # initial pose in odom frame. Only work if rtabmap odom is enabled.
                #  'proj_max_ground_height': 1.5,  # the algorithm ignore points that are under this threshold while projection
                #  'proj_max_ground_angle': 20.0, # proj_max_ground_angle means mapping maximum angle between point's normal to ground's normal to label it as ground. Points with higher angle difference are considered as obstacles.
                #  'proj_min_cluster_size': 20, #  mapping minimum cluster size to project the points. The distance between clusters is defined by 2*grid_cell_size
                 # internal parameters should be string
                 'Optimizer/PriorsIgnored': 'true', # [Optimize a graph with no prior?]
                 'Reg/Force3DoF': 'True',
                #  'Optimizer/Slam2D': 'True',
                 'Grid/CellSize': '0.05',  # grid cell size in meters                 
                #  'Marker/Priors': '"1 0.360, -4.395, 0.228 1.556, -0.022, 3.117|2 -1.495 1.605 4.91 0 0 3.14159"',
                #  'Marker/PriorsVarianceLinear': '0.001',
                #  'Marker/PriorsVarianceAngular': '0.001',
                #  'Grid/FootprintLength': "0.5",
                #  'Grid/FootprintWidth': "0.5",
                #  'Grid/FootprintHeight': "0.15",
                 'Grid/NormalsSegmentation': 'True',  # [Segment ground from obstacles using point normals, otherwise a fast passthrough is used.]
                #  'Grid/MaxGroundHeight': "0.3",   #[Maximum ground height (0=disabled). Should be set if "Grid/NormalsSegmentation" is false.]
                 'Grid/MinClusterSize': "10", # [[Grid/NormalsSegmentation=true] Minimum cluster size to project the points.]
                 'Grid/MaxGroundAngle': "45", # [[Grid/NormalsSegmentation=true] Maximum angle (degrees) between point's normal to ground's normal to label it as ground. Points with higher angle difference are considered as obstacles.]
                 'Grid/ClusterRadius': "0.1", # [[Grid/NormalsSegmentation=true] Cluster maximum radius.]
                 'Grid/FlatObstacleDetected': "true", # [[Grid/NormalsSegmentation=true] Flat obstacles detected.]
                 'Grid/RangeMax': "5.0",  # [Maximum range from sensor. 0=inf.]
                 'GridGlobal/FootprintRadius': "0.2", # [Footprint radius (m) used to clear all obstacles under the graph.]
                
                 }]

    remappings=[
                # ('imu', 'sensors/camera_0/stereolabs_zed/imu/data'),
                ('/tf','tf'),
                ('/tf_static','tf_static'),
                ('apriltag/detections', 'apriltag_detections'),
                ('gps/fix', 'sensors/gps_2/fix'),
    ]

    if LaunchConfiguration('use_zed_odometry').perform(context) in ["True", "true"]:
        remappings.append(('odom', '/j100_0921/sensors/camera_0/stereolabs_zed/odom'))
    else:
        parameters.append({'subscribe_odom_info': True})
    
    return [
        # Sync rgb/depth/camera_info together
        Node(   
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=parameters,
            namespace='j100_0921',
            remappings=[('rgb/image', 'sensors/camera_0/stereolabs_zed/rgb/image_rect_color'),
                        ('rgb/camera_info', 'sensors/camera_0/stereolabs_zed/rgb/camera_info'),
                        ('depth/image', 'sensors/camera_0/stereolabs_zed/depth/depth_registered'),
                        ]),

        # Visual odometry
        # Node(
        #     package='rtabmap_odom', executable='rgbd_odometry', output='screen',
        #     condition=UnlessCondition(LaunchConfiguration('use_zed_odometry')),
        #     parameters=parameters,
        #     namespace='j100_0921',
        #     remappings=remappings,),

        # VSLAM
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            namespace='j100_0921',            
            arguments=['-d'],  #'--delete_db_on_start'
            ),
    ]


def generate_launch_description():
    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_zed_odometry', default_value='true',
            description='Use zed\'s computed odometry instead of using rtabmap\'s odometry.'),
        
        DeclareLaunchArgument(
            'camera_model', default_value='zed2i',
            description="[REQUIRED] The model of the camera. Using a wrong camera model can disable camera features. Valid choices are: ['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'virtual']"),
        
        OpaqueFunction(function=launch_setup)
    ])
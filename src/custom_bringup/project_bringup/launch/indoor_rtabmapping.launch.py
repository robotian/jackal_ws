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
                 'odom_frame_id': 'odom',
                 'approx_sync_max_interval': 0.01,
                #  'imu_topic': 'sensors/camera_0/stereolabs_zed/imu/data',
                 'wait_imu_to_init':False,
                 'tag_topic': 'apriltag_detections',
                 'tag_linear_variance': 0.001,
                 'tag_angular_variance': 0.01,
                 'database_path': '~/.ros/indoor_test_scene3.db',
                #  'gps_topic': 'sensors/gps_2/fix',

                 # internal parameters should be string
                 'Reg/Force3DoF': 'True',
                #  'Optimizer/Slam2D': 'True',
                 'Grid/CellSize': '0.05',  # grid cell size in meters
                 'Optimizer/PriorsIgnored': 'False', # [Optimize a graph with no prior?]
                 'Marker/Priors': '"1 1.075 2.2638  0.320 1.5708 0.0 0.0|2 -0.2272 -2.707 0.472 0 0 3.141592|9 -1.37 2.835 0.59 1.5708   0.0 0.0"',
                #  'Marker/Priors': '"1 1.143 2.286  0.320 0.0 0.0 0.0|2 -0.254 -2.743 0.45 0 0 3.141592"',
                 'Marker/PriorsVarianceLinear': '0.001',
                 'Marker/PriorsVarianceAngular': '0.01',
                #  'Grid/FootprintLength': "0.5",
                #  'Grid/FootprintWidth': "0.5",
                #  'Grid/FootprintHeight': "0.15",
                 'Grid/NormalsSegmentation': 'True',  # [Segment ground from obstacles using point normals, otherwise a fast passthrough is used.]
                #  'Grid/MaxGroundHeight': "0.3",   #[Maximum ground height (0=disabled). Should be set if "Grid/NormalsSegmentation" is false.]
                 'Grid/MinClusterSize': "10", # [[Grid/NormalsSegmentation=true] Minimum cluster size to project the points.]
                 'Grid/MaxGroundAngle': "25", # [[Grid/NormalsSegmentation=true] Maximum angle (degrees) between point's normal to ground's normal to label it as ground. Points with higher angle difference are considered as obstacles.]
                 'Grid/ClusterRadius': "0.1", # [[Grid/NormalsSegmentation=true] Cluster maximum radius.]
                 'Grid/FlatObstacleDetected': "true", # [[Grid/NormalsSegmentation=true] Flat obstacles detected.]
                 'Grid/RangeMax': "5.0",  # [Maximum range from sensor. 0=inf.]
                 'Grid/MaxObstacleHeight': "0.8",
                 'GridGlobal/FootprintRadius': "0.2", # [Footprint radius (m) used to clear all obstacles under the graph.]

                # for localization only mode, uncomment the following three lines and disable the '--delete_db_on_start' launch argument
                 'localization':True,
                 'Mem/IncrementalMemory': 'False', # do not forget to set localization to true
                 'Mem/InitWMWithAllNodes': 'True', # load all nodes in working
                 }]

    remappings=[
                ('imu', 'sensors/camera_0/stereolabs_zed/imu/data'),
                ('/tf','tf'),
                ('/tf_static','tf_static'),
                ('apriltag/detections', 'apriltag_detections'),
                # ('gps/fix', 'sensors/gps_2/fix'),
    ]

    if LaunchConfiguration('use_zed_odometry').perform(context) in ["True", "true"]:
        remappings.append(('odom', 'sensors/camera_0/stereolabs_zed/odom'))
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
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            condition=UnlessCondition(LaunchConfiguration('use_zed_odometry')),
            parameters=parameters,
            namespace='j100_0921',
            remappings=remappings,),

        # VSLAM
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            namespace='j100_0921',
            # arguments=['--delete_db_on_start'],  #'--delete_db_on_start' or '-d'
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
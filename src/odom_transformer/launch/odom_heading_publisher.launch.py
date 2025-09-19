from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='/j100_0921')

    # Declare the use_sim_time argument
    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    # Create the node
    odom_heading_publisher_node = Node(
        package='odom_transformer',  # Replace with your actual package name
        # executable='odom_transformer_node',  # The name of your Python script without the .py extension
        executable='odom_heading_publisher',  # The name of your Python script without the .py extension
        name='odom_heading_publisher',
        output='screen',
        namespace=namespace,
        # remappings=[('/tf','tf'),('/tf_static','tf_static') ],
        # - `front_odom_topic`: ROS topic for front GPS odometry (default: /gps/front/odom)
        # - `rear_odom_topic` : ROS topic for rear GPS odometry (default: /gps/rear/odom)
        # - `parent_frame_id` : Frame ID of the parent frame (default: map)
        # - `child_frame_id`  : Frame ID of the child frame (default: odom)
        parameters=[
            # {'use_sim_time': LaunchConfiguration('use_sim_time')},     
            {'front_odom_topic': '/j100_0921/sensors/gps_1/odom'},               
            {'rear_odom_topic': '/j100_0921/sensors/gps_2/odom'},               
            {'parent_frame_id': 'map'},               
            {'child_frame_id': 'odom'},               
            ]
    )

    # Return the LaunchDescription
    return LaunchDescription([
        arg_namespace,
        # arg_use_sim_time,
        # static_tf_publisher_node,
        odom_heading_publisher_node
    ])

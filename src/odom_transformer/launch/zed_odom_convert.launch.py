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

    # static transform publisher
    # static_tf_publisher_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='odom_tf_publisher',
    #     output='screen',
    #     namespace='husky1',
    #     arguments=['-0.37', '0', '-0.22', '0', '0', '0', '1', 'odom_zed', 'odom'],
    #     remappings=[('/tf','tf'),('/tf_static','tf_static') ],
    # )

    # Create the node
    odom_publisher_node = Node(
        package='odom_transformer',  # Replace with your actual package name
        # executable='odom_transformer_node',  # The name of your Python script without the .py extension
        executable='odom_transformer_ekf_node',  # The name of your Python script without the .py extension
        name='zed_odom_transform',
        output='screen',
        namespace=namespace,
        remappings=[('/tf','tf'),('/tf_static','tf_static') ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},     
                    {'target_odom_topic': '/j100_0921/sensors/camera_0/stereolabs_zed/odom'},               
                    ]
    )

    # Return the LaunchDescription
    return LaunchDescription([
        arg_namespace,
        arg_use_sim_time,
        # static_tf_publisher_node,
        odom_publisher_node
    ])

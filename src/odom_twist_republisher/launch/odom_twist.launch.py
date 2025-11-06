import os
import yaml  # Import the YAML library
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction

def launch_setup(context: LaunchContext):


    # Create the node action, passing the loaded dictionary.
    # The `parameters` argument can take a list of dictionaries.
    # The structure loaded from YAML ({node_name: {ros__parameters: ...}})
    # is exactly what the node expects.
    odomtwist_repub_node = Node(
        package='odom_twist_republisher',
        executable='odom_twist_repub_node',
        name='twist_repub_node_1',  # This must match the name in the param file
        output='screen',
        parameters=[
            {'odom_topic': '/j100_0921/odom_converted'},
            {'twist_topic': '/base_link_twist_odom'},
        ]  # Pass the loaded dictionary
    )

    maptwist_repub_node = Node(
        package='odom_twist_republisher',
        executable='odom_twist_repub_node',
        name='twist_repub_node_2',  # This must match the name in the param file
        output='screen',
        parameters=[
            {'odom_topic': '/j100_0921/rigidbody_1/odom'},
            {'twist_topic': '/base_link_twist_map'},
        ]  # Pass the loaded dictionary
    )

    return [odomtwist_repub_node, maptwist_repub_node]

def generate_launch_description():
    # Use OpaqueFunction to defer node creation
    opaque_function = OpaqueFunction(function=launch_setup)

    return LaunchDescription([
        opaque_function
    ])


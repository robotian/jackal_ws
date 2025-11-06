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
    cmdVelScaler_node = Node(
        package='misc_tools_cpp',
        executable='cmdVelScalerNode',
        name='cmdVelScalerNode',  # This must match the name in the param file
        output='screen',
        namespace='j100_0921',
        parameters=[
            {'scale_linear': 1.125},
            {'scale_angular': 0.87},
        ]  # Pass the loaded dictionary
    )


    return [ cmdVelScaler_node]

def generate_launch_description():
    # Use OpaqueFunction to defer node creation
    opaque_function = OpaqueFunction(function=launch_setup)

    return LaunchDescription([
        opaque_function
    ])


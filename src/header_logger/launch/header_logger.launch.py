import os
import yaml  # Import the YAML library
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction

def launch_setup(context: LaunchContext):
    """
    This function is called by OpaqueFunction to resolve
    the LaunchConfiguration and create the Node.
    """
    # Resolve the LaunchConfiguration to a concrete string path
    params_file_path = context.perform_substitution(LaunchConfiguration('params_file'))

    # Load the parameters from the YAML file into a dictionary
    try:
        with open(params_file_path, 'r') as f:
            params_data = yaml.safe_load(f)
            print(f"Loading parameter file {params_file_path}")
    except Exception as e:
        # Log an error if loading fails
        print(f"Error loading parameter file {params_file_path}: {e}")
        return [] # Return empty list to launch nothing on failure

    # Create the node action, passing the loaded dictionary.
    # The `parameters` argument can take a list of dictionaries.
    # The structure loaded from YAML ({node_name: {ros__parameters: ...}})
    # is exactly what the node expects.
    header_logger_node = Node(
        package='header_logger',
        executable='header_timestamp_logger',
        name='header_timestamp_logger',  # This must match the name in the param file
        output='screen',
        parameters=[params_data]  # Pass the loaded dictionary
    )

    return [header_logger_node]

def generate_launch_description():
    # Get the path to the package share directory
    pkg_share = get_package_share_directory('header_logger')
    
    # Define the path to the parameter file
    default_param_file = os.path.join(pkg_share, 'config', 'params.yaml')

    # Declare the launch argument
    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_param_file,
        description='Full path to the parameter file to use.'
    )

    # Use OpaqueFunction to defer node creation
    opaque_function = OpaqueFunction(function=launch_setup)

    return LaunchDescription([
        declare_params_file_arg,
        opaque_function
    ])


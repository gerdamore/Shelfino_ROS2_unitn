import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import launch.logging
import logging

def print_env(context):
    print(__file__)
    for key in context.launch_configurations.keys():
        print("\t", key, context.launch_configurations[key])
    return

def generate_launch_description():
    # launch.logging.launch_config.level = logging.DEBUG
    
    map_env_pkg = get_package_share_directory('path_planning')

    # map_env_params_file_path = os.path.join(map_env_pkg, 'config', 'map_config.yaml')
    # if (not os.path.exists(map_env_params_file_path)):
    #     raise Exception("[{}] Map config file `{}` does not exist".format(__file__, map_env_params_file_path))

    # General arguments
    # map_env_params_file = LaunchConfiguration('map_env_params_file', default=map_env_params_file_path)

    # # Declare LaunchArguments for exposing launching arguments
    # launch_args = [
    #     DeclareLaunchArgument(
    #         'map_env_params_file',
    #         default_value=map_env_params_file_path,
    #         description='Full path to the map_pkg params file to use'
    #     ),
    # ]

    # List of nodes to launch
    nodes = [
        Node (
            package='path_planning',
            executable='Follow_path',
            name='fp',
            output='screen',
        ),

    ]

    # LaunchDescription with the additional launch files
    ld = LaunchDescription()

    for launch_arg in launch_args:
        ld.add_action(launch_arg)

    ld.add_action(OpaqueFunction(function=print_env))
    
    for node in nodes:
        ld.add_action(node)

    return ld

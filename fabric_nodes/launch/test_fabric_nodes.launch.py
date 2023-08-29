# Copyright 2023 U Power Robotics USA, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


"""
This module contains the function `generate_launch_description`.

The configuration of the nodes is defined in a yaml file, either given by the 'CONFIG_FILE_PATH'
environment variable or the default one located in 'param/pass_config.param.yaml'.
A `Config2Nodes` object is created to parse this configuration for environment 'env1', which then
generates the corresponding nodes.
Note: This file should not be removed even in custom, cause it is use to do 'colcon test'
"""


import os

from ament_index_python import get_package_share_directory
from fabric_nodes.config2node import Config2Nodes
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration


def get_nodes(context, *args, **kwargs):
    config_path = LaunchConfiguration('config-path').perform(context)
    environment = LaunchConfiguration('environment').perform(context)

    config2nodes = Config2Nodes(config_path, environment)
    nodes = config2nodes.get_nodes()

    return nodes


def generate_launch_description():
    """
    Generate the LaunchDescription for the fabric_nodes package.

    Returns
    -------
    LaunchDescription: The LaunchDescription object.

    """
    pass_config_path = os.path.join(
        get_package_share_directory('fabric_nodes'),
        'test_config/pass_config.test.yaml'
    )

    config_path_arg = DeclareLaunchArgument(
        name='config-path',
        default_value=pass_config_path,
        description='Path to the configuration file to load'
    )

    environment_arg = DeclareLaunchArgument(
        name='environment',
        default_value='env1',
        description='The environment from the config file from which to launch nodes'
    )

    return LaunchDescription(
      [config_path_arg, environment_arg] + [OpaqueFunction(function=get_nodes)]
    )

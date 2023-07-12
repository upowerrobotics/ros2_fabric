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

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import yaml


def generate_launch_description():
    """
    Generate the LaunchDescription for the fabric_nodes package.

    Returns
    -------
    The LaunchDescription object.

    """
    pkg_share_path = get_package_share_directory('fabric_nodes')
    config_file_path = os.path.join(pkg_share_path, 'param/dummy_config.param.yaml')

    with open(config_file_path, 'r') as file:
        config = yaml.safe_load(file)

    nodes = []

    # Process publish topics
    for env_name, env_config in config['environments'].items():
        if env_name == 'env1':
            for nodename, nodeconfig in env_config['nodes'].items():
                node = Node(
                    package='fabric_nodes',
                    executable='dummy_node_exe',
                    name='fabric_node',
                    namespace=nodename,
                    parameters=[{
                        'root_node': nodeconfig['root_node'],
                        'terminal_node': nodeconfig['terminal_node'],
                        'publish_topics': nodeconfig['publish_topics'],
                        'subscribe_topics': nodeconfig['subscribe_topics'],
                    }],
                    output='screen'
                )
                nodes.append(node)

    return LaunchDescription(nodes)

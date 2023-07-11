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

from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

import os
import yaml


def generate_launch_description():
    pkg_share_path = get_package_share_directory('fabric_nodes')
    config_file_path = os.path.join(pkg_share_path, 'param/dummy_config.param.yaml')

    with open(config_file_path, 'r') as file:
        config = yaml.safe_load(file)

    nodes = []

    # Process publish topics
    for publisher_name, publisher_config in config['ros__parameters']['publish_topics'].items():
        node_name = 'publisher_node_' + publisher_name
        node = Node(
            package='fabric_nodes',
            executable='dummy_node_exe',
            name=node_name,
            parameters=[{
                'root_node': config['ros__parameters']['root_node'],
                'terminal_node': config['ros__parameters']['terminal_node'],
                'publish_topics': {publisher_name: publisher_config}
            }],
            output='screen'
        )
        nodes.append(node)

    # Process subscribe topics
    for subscriber_name, subscriber_config in (
        config['ros__parameters']['subscribe_topics'].items()
    ):
        node_name = subscriber_config['node']
        node = Node(
            package='fabric_nodes',
            executable='dummy_node_exe',
            name='subscriber_node_' + subscriber_name,
            parameters=[{
                'root_node': config['ros__parameters']['root_node'],
                'terminal_node': config['ros__parameters']['terminal_node'],
                'subscribe_topics': {subscriber_name: subscriber_config}
            }],
            output='screen'
        )
        nodes.append(node)

    return LaunchDescription(nodes)

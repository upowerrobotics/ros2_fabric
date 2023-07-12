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
import sys

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import yaml


def check_config_rules(config):
    # Initialize flags for root and terminal nodes
    has_root_publisher = False
    has_terminal_subscriber = False

    for env_name, env_config in config['environments'].items():
        for nodename, nodeconfig in env_config['nodes'].items():
            is_root_node = nodeconfig['root_node']
            is_terminal_node = nodeconfig['terminal_node']

            # Check if a node is both a root node and a terminal node
            if is_root_node and is_terminal_node:
                print(f"Error: Node '{nodename}' cannot be both a root node and a terminal node.")
                return 1

            # Check if a root node has subscribers
            if is_root_node and 'subscribe_topics' in nodeconfig:
                print(f"Error: Root node '{nodename}' should not have subscribers.")
                return 1

            # Check if a terminal node has publishers
            if is_terminal_node and 'publish_topics' in nodeconfig:
                print(f"Error: Terminal node '{nodename}' should not have publishers.")
                return 1

            # Check each publisher for required parameters
            if 'publish_topics' in nodeconfig:
                for topic, params in nodeconfig['publish_topics'].items():
                    param_count = sum(param in params for param in ['bandwidth', 'msg_size', 'msg_frequency'])
                    if param_count != 2:
                        print(f"Error: Publisher '{nodename}' on topic '{topic}' must have exactly 2 of the following parameters: "
                              "'bandwidth', 'msg_size', 'msg_frequency'.")
                        return 1

            # Check if each publisher has a connected subscriber
            """TO DO"""

            # Update root and terminal node flags
            if is_root_node:
                has_root_publisher = True
            if is_terminal_node:
                has_terminal_subscriber = True

    # Check if there is at least one root node and one terminal node
    if not has_root_publisher:
        print("Error: There must be at least one root node with publishers.")
        return 1
    if not has_terminal_subscriber:
        print("Error: There must be at least one terminal node with subscribers.")
        return 1

    # All checks passed
    print("INFO: Config file pass rule check.")
    return 0

def generate_launch_description():
    """
    Check the config rules and Generate the LaunchDescription for the fabric_nodes package.

    Returns
    -------
    The LaunchDescription object.

    """
    pkg_share_path = get_package_share_directory('fabric_nodes')
    config_file_path = os.path.join(pkg_share_path, 'param/dummy_config.param.yaml')
    with open(config_file_path, 'r') as file:
        config = yaml.safe_load(file)
    if (check_config_rules(config)):
        sys.exit()

    nodes = []

    # Process publish topics
    for env_name, env_config in config['environments'].items():
        if env_name == 'env1':
            for nodename, nodeconfig in env_config['nodes'].items():
                publish_topics = nodeconfig.get('publish_topics', {})
                subscribe_topics = nodeconfig.get('subscribe_topics', {})
                node = Node(
                    package='fabric_nodes',
                    executable='dummy_node_exe',
                    name=nodename,
                    namespace=nodename,
                    parameters=[{
                        'root_node': nodeconfig['root_node'],
                        'terminal_node': nodeconfig['terminal_node'],
                        'publish_topics': publish_topics,
                        'subscribe_topics': subscribe_topics,
                    }],
                    output='screen'
                )
                nodes.append(node)

    return LaunchDescription(nodes)

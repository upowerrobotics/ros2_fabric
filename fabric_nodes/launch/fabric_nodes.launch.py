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
    Check the config rules and Generate the LaunchDescription for the fabric_nodes package.

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
    for environment in config['environments']:
        if environment['name'] != 'env1':
            continue
        
        # Process nodes
        for node_config in environment['nodes']:
            node_name = node_config['name']
            root_node = node_config['root_node']
            terminal_node = node_config['terminal_node']
            node_qty = node_config.get('qty', 1)

            # Process publishers
            publish_topics = {}
            if 'publishers' in node_config:
                for publisher in node_config['publishers']:
                    topic_name = publisher['name']
                    publish_topic = {}
                    if 'msg_size' in publisher:
                        publish_topic['msg_size'] = publisher['msg_size']
                    if 'bandwidth' in publisher:
                        publish_topic['bandwidth'] = publisher['bandwidth']
                    if 'msg_frequency' in publisher:
                        publish_topic['msg_frequency'] = publisher['msg_frequency']
                    publisher_qty = publisher.get('qty', 1)
                    if publisher_qty != 1:
                        for num in range(1, publisher_qty + 1):
                            publish_topics[topic_name + '_' + str(num)] = publish_topic
                    else:
                        publish_topics[topic_name] = publish_topic

            # Process subscribers
            subscribe_topics = {}
            if 'subscribers' in node_config:
                for subscriber in node_config['subscribers']:
                    topic_name = subscriber['name']
                    subscribe_topic = {'node': subscriber['node']}
                    subscriber_qty = subscriber.get('qty', 1)
                    if subscriber_qty != 1:
                        for num in range(1, subscriber_qty + 1):
                            subscribe_topics[topic_name + '_' + str(num)] = subscribe_topic
                    else:
                        subscribe_topics[topic_name] = subscribe_topic
            
            if node_qty != 1:
                for num in range(1, node_qty + 1):
                    node = Node(
                        package='fabric_nodes',
                        executable='dummy_node_exe',
                        name=node_name + '_' + str(num),
                        namespace=node_name + '_' + str(num),
                        parameters=[{
                            'root_node': root_node,
                            'terminal_node': terminal_node,
                            'publish_topics': publish_topics,
                            'subscribe_topics': subscribe_topics,
                        }],
                        output='screen'
                    )
                    nodes.append(node)
            else:
                node = Node(
                    package='fabric_nodes',
                    executable='dummy_node_exe',
                    name=node_name,
                    namespace=node_name,
                    parameters=[{
                        'root_node': root_node,
                        'terminal_node': terminal_node,
                        'publish_topics': publish_topics,
                        'subscribe_topics': subscribe_topics,
                    }],
                    output='screen'
                )
                nodes.append(node)

    return LaunchDescription(nodes)

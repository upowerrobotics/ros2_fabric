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

from launch_ros.actions import Node

import yaml


##
# @class Config2Nodes.
# @brief Helper class for processing configuration files and generating nodes.
#
class Config2Nodes:

    ##
    # @brief The contructor for Config2Nodes.
    # @param [in] config_file_path The path of the config file.
    # @param [in] env The environment for processing the configuration.
    #
    def __init__(self, config_file_path, env):
        ## The path of the configuration file.
        self.config_file_path = config_file_path
        ## The environment that is operate in this configuration.
        self.env = env
        ## The list of generate nodes that extracted from configuration.
        self.nodes = []
        ## The configuration that extracted from the path of the configuration file.
        self.config = None

    ##
    # @brief Load the configuration file.
    #
    def load_config(self):
        with open(self.config_file_path, 'r') as file:
            self.config = yaml.safe_load(file)

    ##
    # @brief Validate the loaded configuration.
    #
    def validate_config(self):
        publisher_topics = set()
        subscriber_topics = set()

        for environment in self.config['environments']:
            for node_config in environment['nodes']:
                # Validate node qty rules
                node_qty = node_config.get('qty', 1)
                if node_qty < 1:
                    raise ValueError(f"Invalid node quantity for node '{node_config['name']}'")

                node_publishers = node_config.get('publishers', [])

                for publisher in node_publishers:
                    # Validate publishers qty rules
                    publisher_qty = publisher.get('qty', 1)
                    if publisher_qty < 1:
                        raise ValueError(
                            f'Invalid publisher quantity for publisher '
                            f"{publisher['name']}' in node '{node_config['name']}'"
                        )

                    # Validate publisher parameters
                    parameter_count = sum(parameter in publisher for parameter in [
                        'bandwidth', 'msg_size', 'frequency'])
                    if parameter_count < 2:
                        raise ValueError(
                            f"Publisher '{publisher['name']}' in node "
                            f"'{node_config['name']}' must have at least two of the "
                            f'following parameters: bandwidth, msg_size, frequency'
                        )
                    
                    # Validate QoS settings for publishers
                    qos_depth = publisher.get('QoS_depth')
                    qos_policy = publisher.get('QoS_policy')
                    if qos_depth is None or qos_policy is None:
                        raise ValueError(
                            f"Publisher '{publisher['name']}' must have both QoS_depth and QoS_policy"
                        )
                    if qos_depth < 0:
                        raise ValueError(
                            f"QoS_depth for publisher '{publisher['name']}' must be >= 0"
                        )
                    if qos_policy not in ['reliable', 'best_effort']:
                        raise ValueError(
                            f"QoS_policy for publisher '{publisher['name']}' must be either 'reliable' or 'best_effort'"
                        )

                    # Store publisher_topics
                    for i in range(1, node_qty + 1):
                        base_node_name = node_config['name']
                        node_name = base_node_name if node_qty == 1 else f'{base_node_name}_{i}'

                        for j in range(1, publisher_qty + 1):
                            base_publisher_name = publisher['name']
                            publisher_name = (
                                base_publisher_name if publisher_qty == 1
                                else f'{base_publisher_name}_{j}')
                            publisher_topics.add((node_name, publisher_name))

                node_subscribers = node_config.get('subscribers', [])

                for subscriber in node_subscribers:
                    # Validate QoS settings for subscribers
                    qos_depth = subscriber.get('QoS_depth')
                    qos_policy = subscriber.get('QoS_policy')
                    if qos_depth is None or qos_policy is None:
                        raise ValueError(
                            f"Subscriber '{subscriber['name']}' must have both QoS_depth and QoS_policy"
                        )
                    if qos_depth < 0:
                        raise ValueError(
                            f"QoS_depth for subscriber '{subscriber['name']}' must be >= 0"
                        )
                    if qos_policy not in ['reliable', 'best_effort']:
                        raise ValueError(
                            f"QoS_policy for subscriber '{subscriber['name']}' must be either 'reliable' or 'best_effort'"
                        )

                    # Store subscriber_topics
                    for i in range(1, node_qty + 1):
                        base_subscriber_node_name = subscriber['node']
                        subscriber_node_name = (
                            base_subscriber_node_name if node_qty == 1 else
                            f'{base_subscriber_node_name}_{i}')
                        subscriber_topics.add(
                            (subscriber_node_name, subscriber['name'], node_config['name']))

                # Validate root_node and terminal_node rules
                root_node = node_config.get('root_node', False)
                terminal_node = node_config.get('terminal_node', False)
                if root_node and terminal_node:
                    raise ValueError(
                        f"Node '{node_config['name']}' "
                        f'cannot be both a terminal node and a root node'
                    )
                if root_node and 'subscribers' in node_config:
                    raise ValueError(
                        f"Root node '{node_config['name']}' cannot have subscribers"
                    )
                if terminal_node and 'publishers' in node_config:
                    raise ValueError(
                        f"Terminal node '{node_config['name']}' cannot have publishers"
                    )
                if ((not root_node and not terminal_node) and
                        ('subscribers' not in node_config or 'publishers' not in node_config)):
                    raise ValueError(
                        f"Node '{node_config['name']}' must contain both publishers and "
                        f'subscribers since it is not a root or terminal node'
                    )

        # Validate publishers-subscribers connections
        for publisher_topic in publisher_topics:
            pub_connected = False
            for subscriber_topic in subscriber_topics:
                if (publisher_topic[0] == subscriber_topic[0] and
                        publisher_topic[1] == subscriber_topic[1]):
                    pub_connected = True
            if not pub_connected:
                raise ValueError(
                    f"Publisher '{publisher_topic[1]}' in node "
                    f"'{publisher_topic[0]}' is not connected properly"
                )
        for subscriber_topic in subscriber_topics:
            sub_connected = False
            for publisher_topic in publisher_topics:
                if (subscriber_topic[0] == publisher_topic[0] and
                        subscriber_topic[1] == publisher_topic[1]):
                    sub_connected = True
            if not sub_connected:
                raise ValueError(
                    f"Subscriber '{subscriber_topic[1]}' in node "
                    f"'{subscriber_topic[2]}' is not connected properly"
                )
    ##
    # @brief Process publishers in the node configuration.
    # @param [in] node_config The configuration of a specific node.
    # @return A dictionary containing the processed publisher topics.
    #
    def process_publishers(self, node_config):
        publish_topics = {}
        publishers = node_config.get('publishers', [])

        for publisher in publishers:
            topic_name = publisher['name']
            publish_topic = {}

            if 'msg_size' in publisher:
                publish_topic['msg_size'] = publisher['msg_size']
            if 'bandwidth' in publisher:
                publish_topic['bandwidth'] = publisher['bandwidth']
            if 'frequency' in publisher:
                publish_topic['frequency'] = publisher['frequency']

            publish_topic['QoS_depth'] = publisher['QoS_depth']
            publish_topic['QoS_policy'] = publisher['QoS_policy']

            publisher_qty = publisher.get('qty', 1)

            for num in range(1, publisher_qty + 1):
                key = topic_name if publisher_qty == 1 else f'{topic_name}_{num}'
                publish_topics[key] = publish_topic

        return publish_topics

    ##
    # @brief Process subscribers in the node configuration.
    # @param [in] node_config The configuration of a specific node.
    # @return A dictionary containing the processed subscriber topics.
    #
    def process_subscribers(self, node_config):
        subscribe_topics = {}
        subscribers = node_config.get('subscribers', [])

        for subscriber in subscribers:
            topic_name = subscriber['name']
            subscribe_topic = {}
            subscribe_topic['node'] = subscriber['node']
            subscribe_topic['QoS_depth'] = subscriber['QoS_depth']
            subscribe_topic['QoS_policy'] = subscriber['QoS_policy']

            subscribe_topics[f"{subscriber['node']}/{topic_name}"] = subscribe_topic

        return subscribe_topics

    ##
    # @brief Generate a node based on the provided configuration.
    # @param [in] node_config The configuration of a specific node.
    # @param [in] publish_topics The processed publisher topics.
    # @param [in] subscribe_topics The processed subscriber topics.
    #
    def generate_node(self, node_config, publish_topics, subscribe_topics):
        node_qty = node_config.get('qty', 1)
        node_name = node_config['name']
        root_node = node_config['root_node']
        terminal_node = node_config['terminal_node']

        for num in range(1, node_qty + 1):
            if node_qty != 1:
                subscribe_topics_qty = {
                    f'{namespace}_{num}/{topic}': {'node': f"{value['node']}_{num}",
                                                   'QoS_depth': value['QoS_depth'], 'QoS_policy': value['QoS_policy']}
                    for key, value in subscribe_topics.items()
                    for namespace, topic in [key.split('/')]
                }
                node_name_with_num = f'{node_name}_{num}'
            else:
                subscribe_topics_qty = subscribe_topics
                node_name_with_num = node_name

            node = self.create_node(node_name_with_num, root_node, terminal_node,
                                    publish_topics, subscribe_topics_qty)
            self.nodes.append(node)

    ##
    # @brief Create a ROS node object.
    # @param [in] name The name of the node.
    # @param [in] root_node Whether the node is a root node.
    # @param [in] terminal_node Whether the node is a terminal node.
    # @param [in] publish_topics The processed publisher topics.
    # @param [in] subscribe_topics The processed subscriber topics.
    # @return The created ROS node object.
    #
    def create_node(self, name, root_node, terminal_node, publish_topics, subscribe_topics):
        node = Node(
            package='fabric_nodes',
            executable='dummy_node_exe',
            name=name,
            namespace=name,
            parameters=[{
                'root_node': root_node,
                'terminal_node': terminal_node,
                'publish_topics': publish_topics,
                'subscribe_topics': subscribe_topics,
            }],
            arguments=['--ros-args', '--log-level', 'DEBUG'],
            output='log'
        )
        return node

    ##
    # @brief Get the generated nodes based on the configuration.
    # @return A list of the generated ROS node objects.
    #
    def get_nodes(self):
        self.load_config()
        self.validate_config()

        for environment in self.config['environments']:
            if environment['name'] != self.env:
                continue
            for node_config in environment['nodes']:
                publish_topics = self.process_publishers(node_config)
                subscribe_topics = self.process_subscribers(node_config)
                self.generate_node(node_config, publish_topics, subscribe_topics)

        return self.nodes

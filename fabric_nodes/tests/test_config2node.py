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
from fabric_nodes.config2node import Config2Nodes
import pytest


def test_error_configuration():
    """
    Test different error configurations for the fabric_nodes package.

    This function tests several error configuration scenarios in which a ValueError is expected to
    be raised. For each configuration file in the 'test_cases' dictionary, it creates a
    Config2Nodes object and calls the 'get_nodes' method, expecting a ValueError to be thrown. It
    then asserts that the actual error message includes the expected error message.
    """
    test_cases = {
        'error_param_pub.test.yaml': ('must have at least two '
                                      'of the following parameters: '
                                      'bandwidth, msg_size, frequency'),
        'error_pub_connection_qty.test.yaml': 'is not connected',
        'error_pub_connection.test.yaml': 'is not connected',
        'error_qty_node.test.yaml': 'Invalid node quantity',
        'error_qty_pub.test.yaml': 'Invalid publisher quantity',
        'error_missing_pubs.test.yaml': 'must contain both publishers and subscribers',
        'error_missing_subs.test.yaml': 'must contain both publishers and subscribers',
        'error_root_terminal.test.yaml': 'cannot be both a terminal node and a root node',
        'error_root.test.yaml': 'cannot have subscribers',
        'error_sub_connection_qty.test.yaml': 'is not connected',
        'error_sub_connection.test.yaml': 'is not connected',
        'error_terminal.test.yaml': 'cannot have publishers',
        'error_pub_qos_policy.yaml': 'QoS_policy for publisher',
        'error_sub_qos_policy.yaml': 'QoS_policy for subscriber',
        'error_pub_qos_depth.yaml': 'QoS_depth for publisher',
        'error_sub_qos_depth.yaml': 'QoS_depth for subscriber'
    }

    for config_file, expected_error in test_cases.items():
        config_file_path = os.path.join(
            get_package_share_directory('fabric_nodes'),
            'test_config/',
            config_file
        )

        config2nodes = Config2Nodes(config_file_path, 'env1')

        with pytest.raises(ValueError) as excinfo:
            config2nodes.get_nodes()

        assert (expected_error in str(excinfo.value))

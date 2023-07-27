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
        'error_param_pub.param.yaml': (f'must have at least two '
                                       f'of the following parameters: '
                                       f'bandwidth, msg_size, frequency'),
        'error_pub_connection_qty.param.yaml': 'publisher is not connected',
        'error_pub_connection.param.yaml': 'publisher is not connected',
        'error_qty_node.param.yaml': 'node qty is not a valid number',
        'error_qty_pub.param.yaml': 'publisher qty is not a valid number',
        'error_qty_sub.param.yaml': 'subscriber qty is not a valid number',
        'error_root_terminal.param.yaml': 'root node can not be terminal node',
        'error_root.param.yaml': 'root node can not have subscribers',
        'error_sub_connection_qty.param.yaml': 'subsciber is not connected',
        'error_sub_connetion.param.yaml': 'subsciber is not connected',
        'error_terminal.param.yaml': 'terminal node can not have publihsers',
    }

    for config_file, expected_error in test_cases.items():
        config_file_path = os.path.join(
            get_package_share_directory('fabric_nodes'),
            'param/error_test_configs/',
            config_file
        )

        config2nodes = Config2Nodes(config_file_path, 'env1')

        with pytest.raises(ValueError) as excinfo:
            config2nodes.get_nodes()

        assert (expected_error + str(excinfo.value))

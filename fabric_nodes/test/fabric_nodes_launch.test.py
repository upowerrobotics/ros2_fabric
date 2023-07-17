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
import unittest

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import pytest
from fabric_nodes.config2node import Config2Nodes


@pytest.mark.launch_test
def generate_test_description():

    config_file_path = os.path.join(
        get_package_share_directory('fabric_nodes'),
        'param/pass_config.param.yaml'
    )
    launch_file_path = os.path.join(
        get_package_share_directory('fabric_nodes'),
        'launch/fabric_nodes.launch.py'
    )

    os.environ['CONFIG_FILE_PATH'] = config_file_path
    launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_path)
    )

    context = {}

    return LaunchDescription([
        launch_description,
        # Start tests right away - no need to wait for anything
        launch_testing.actions.ReadyToTest()]
    ), context


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_info):
        # Check that the process exits with code -2 (termination request)
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[-2, -6, -15]
        )

    def test_process_creation(self, proc_output):
        # Get the process names that generated IO
        process_names = proc_output.process_names()

        # Check if the expected topics are present
        dummy_process = ['dummy_node_exe-1', 'dummy_node_exe-2', 'dummy_node_exe-3',
                         'dummy_node_exe-4', 'dummy_node_exe-5', 'dummy_node_exe-6']

        for process_name in process_names:
            assert process_name in dummy_process, (
                f'{process_name} was not found in dummy_process.'
            )

    # Test publisher param config checker
    def test_error_param_pub(self):
        config_file_path = os.path.join(
            get_package_share_directory('fabric_nodes'),
            'param/error_test_configs/error_param_pub.param.yaml'
        )

        config2nodes = Config2Nodes(config_file_path, 'env1')

        with pytest.raises(ValueError) as excinfo:
            config2nodes.get_nodes()

        assert (f'must have at least two of the following parameters: '
                f'bandwidth, msg_size, msg_frequency in ' + str(excinfo.value))

    # Test publisher connection if different qty
    def test_error_pub_connection_qty(self):
        config_file_path = os.path.join(
            get_package_share_directory('fabric_nodes'),
            'param/error_test_configs/error_pub_connection_qty.param.yaml'
        )

        config2nodes = Config2Nodes(config_file_path, 'env1')

        with pytest.raises(ValueError) as excinfo:
            config2nodes.get_nodes()

        assert (f'publisher is not connected in ' + str(excinfo.value))

    # Test publisher connection if same qty
    def test_error_pub_connection(self):
        config_file_path = os.path.join(
            get_package_share_directory('fabric_nodes'),
            'param/error_test_configs/error_pub_connection.param.yaml'
        )

        config2nodes = Config2Nodes(config_file_path, 'env1')

        with pytest.raises(ValueError) as excinfo:
            config2nodes.get_nodes()

        assert (f'publisher is not connected in ' + str(excinfo.value))

    # Test qty node
    def test_error_qty_node(self):
        config_file_path = os.path.join(
            get_package_share_directory('fabric_nodes'),
            'param/error_test_configs/error_qty_node.param.yaml'
        )

        config2nodes = Config2Nodes(config_file_path, 'env1')

        with pytest.raises(ValueError) as excinfo:
            config2nodes.get_nodes()

        assert (f'node qty is not a valid number in ' + str(excinfo.value))

    # Test qty publisher
    def test_error_qty_publisher(self):
        config_file_path = os.path.join(
            get_package_share_directory('fabric_nodes'),
            'param/error_test_configs/error_qty_pub.param.yaml'
        )

        config2nodes = Config2Nodes(config_file_path, 'env1')

        with pytest.raises(ValueError) as excinfo:
            config2nodes.get_nodes()

        assert (f'publisher qty is not a valid number in ' + str(excinfo.value))

    # Test qty subscriber
    def test_error_qty_subscriber(self):
        config_file_path = os.path.join(
            get_package_share_directory('fabric_nodes'),
            'param/error_test_configs/error_qty_sub.param.yaml'
        )

        config2nodes = Config2Nodes(config_file_path, 'env1')

        with pytest.raises(ValueError) as excinfo:
            config2nodes.get_nodes()

        assert (f'subscriber qty is not a valid number in ' + str(excinfo.value))

    # Test root terminal node
    def test_error_root_terminal(self):
        config_file_path = os.path.join(
            get_package_share_directory('fabric_nodes'),
            'param/error_test_configs/error_root_terminal.param.yaml'
        )

        config2nodes = Config2Nodes(config_file_path, 'env1')

        with pytest.raises(ValueError) as excinfo:
            config2nodes.get_nodes()

        assert (f'root node can not be terminal node in ' + str(excinfo.value))

    # Test root node
    def test_error_root(self):
        config_file_path = os.path.join(
            get_package_share_directory('fabric_nodes'),
            'param/error_test_configs/error_root.param.yaml'
        )

        config2nodes = Config2Nodes(config_file_path, 'env1')

        with pytest.raises(ValueError) as excinfo:
            config2nodes.get_nodes()

        assert (f'root node can not have subscribers in ' + str(excinfo.value))

    # Test subscriber connection if different qty
    def test_error_sub_connection_qty(self):
        config_file_path = os.path.join(
            get_package_share_directory('fabric_nodes'),
            'param/error_test_configs/error_sub_connection_qty.param.yaml'
        )

        config2nodes = Config2Nodes(config_file_path, 'env1')

        with pytest.raises(ValueError) as excinfo:
            config2nodes.get_nodes()

        assert (f'subsciber is not connected in ' + str(excinfo.value))

    # Test subscriber connection if same qty
    def test_error_sub_connetion(self):
        config_file_path = os.path.join(
            get_package_share_directory('fabric_nodes'),
            'param/error_test_configs/error_sub_connetion.param.yaml'
        )

        config2nodes = Config2Nodes(config_file_path, 'env1')

        with pytest.raises(ValueError) as excinfo:
            config2nodes.get_nodes()

        assert (f'subsciber is not connected in ' + str(excinfo.value))

    # Test terminal node
    def test_error_terminal(self):
        config_file_path = os.path.join(
            get_package_share_directory('fabric_nodes'),
            'param/error_test_configs/error_terminal.param.yaml'
        )

        config2nodes = Config2Nodes(config_file_path, 'env1')

        with pytest.raises(ValueError) as excinfo:
            config2nodes.get_nodes()

        assert (f'terminal node can not have publihsers in ' + str(excinfo.value))

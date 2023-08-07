## @file
# This file contains tests for verifying the correct launch of nodes in a ROS2 environment.
#
# Tests are executed at launch time and post-shutdown. They ensure that the required processes
# are properly launched and that they terminate with expected exit codes.

## @package ros2_nodes_test
# Documentation for the ROS2 nodes test package.

## @package TestProcessOutput
# Documentation for the TestProcessOutput class.

## @package generate_test_description
# Documentation for the generate_test_description function.

## @package fabric_nodes_test
# Documentation for the fabric_nodes_test module.

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


## Generate a launch description for the nodes under test.
@pytest.mark.launch_test
def generate_test_description():
    """
    Generate a launch description for the nodes under test.

    Returns
    -------
        LaunchDescription: A launch description with the paths to the node launch file
                        and the config file. ReadyToTest action is also included
                        to start tests immediately.
        dict: A context dictionary containing the path to the config file.

    """
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
        launch_testing.actions.ReadyToTest()]
    ), context


## A test case class for validating the process output after the nodes have been shutdown.
@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    """TestCase for validating the process output after the nodes have been shutdown."""

    def test_process_creation(self, proc_output):
        """
        Test if the processes launched include the dummy processes.

        Args
        ----
        proc_output: The output of the processes under test.

        Raises
        ------
        AssertionError: If a process name from the output is
                        not found in the list of dummy processes.

        """
        process_names = proc_output.process_names()
        dummy_process = ['dummy_node_exe-1', 'dummy_node_exe-2', 'dummy_node_exe-3',
                         'dummy_node_exe-4', 'dummy_node_exe-5', 'dummy_node_exe-6']
        for process_name in process_names:
            assert process_name in dummy_process, (
                f'{process_name} was not found in dummy_process.'
            )

    def test_exit_code(self, proc_info):
        """
        Test if the process exits with acceptable exit codes.

        Args
        ----
        proc_info: Information about the process under test.

        Raises
        ------
        AssertionError: If the process exit code is not among the allowable exit codes.

        """
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -2, -6, -15]
        )

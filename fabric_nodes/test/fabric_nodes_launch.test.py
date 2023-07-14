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

import pytest
import launch_testing

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


@pytest.mark.launch_test
def generate_test_description():

    launch_file_path = os.path.join(
        get_package_share_directory('fabric_nodes'),
        'launch',
        'fabric_nodes.launch.py'
    )

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

    def test_exit_code(self, proc_output, proc_info):
        # Check that process exits with code -2 code: termination request, sent to the program
        launch_testing.asserts.assertExitCodes(
            proc_info,
            [-2, -6, -15]
        )

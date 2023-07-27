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
import signal
import subprocess
import time


def test_node_list():
    """
    Test the availability of the expected nodes in the ROS2 environment.

    This function launches a subprocess which starts the 'fabric_nodes.launch.py' ROS2 launch
    file. It then retrieves the list of active nodes using 'ros2 node list', and checks if the
    expected nodes are in the list. The launch subprocess is then terminated, and the function
    finally asserts that all the expected nodes were processed.
    """
    expect_nodes = [
        '/node1_1/node1_1',
        '/node1_2/node1_2',
        '/node2_1/node2_1',
        '/node2_2/node2_2',
        '/node3_1/node3_1',
        '/node3_2/node3_2'
    ]
    launch_str = 'ros2 launch fabric_nodes fabric_nodes.launch.py'
    node_list_str = 'ros2 node list'

    # Start the launch subprocess
    launch_process = subprocess.Popen(launch_str, shell=True, preexec_fn=os.setpgrp)
    time.sleep(1)

    # Get node list
    node_list_process = subprocess.run(node_list_str, shell=True, capture_output=True, text=True)
    node_list = node_list_process.stdout.strip().splitlines()

    # Terminate the launch subprocess
    os.killpg(os.getpgid(launch_process.pid), signal.SIGINT)
    launch_process.wait()

    # Check node list
    for expect_node in expect_nodes:
        assert expect_node in node_list, f"'expect_node '{expect_node}' not processed"

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


def test_topic_list():
    """
    Test the availability of the expected topics in the ROS2 environment.

    This function launches a subprocess which starts the 'fabric_nodes.launch.py' ROS2 launch
    file. It then retrieves the list of active topics using 'ros2 topic list', and checks if the
    expected topics are in the list. The launch subprocess is then terminated, and the function
    finally asserts that all the expected topics were processed.
    """
    expect_topics = [
        '/node1_1/topic_1M',
        '/node1_1/topic_2M_1',
        '/node1_1/topic_2M_2',
        '/node1_1/topic_500K_1',
        '/node1_1/topic_500K_2',
        '/node1_1/topic_500K_3',
        '/node1_2/topic_1M',
        '/node1_2/topic_2M_1',
        '/node1_2/topic_2M_2',
        '/node1_2/topic_500K_1',
        '/node1_2/topic_500K_2',
        '/node1_2/topic_500K_3',
        '/node2_1/topic_1M_1',
        '/node2_1/topic_1M_2',
        '/node2_1/topic_1M_3',
        '/node2_1/topic_1M_4',
        '/node2_1/topic_2M_1',
        '/node2_1/topic_2M_2',
        '/node2_1/topic_2M_3',
        '/node2_1/topic_2M_4',
        '/node2_1/topic_2M_5',
        '/node2_1/topic_500K_1',
        '/node2_1/topic_500K_2',
        '/node2_1/topic_500K_3',
        '/node2_1/topic_500K_4',
        '/node2_1/topic_500K_5',
        '/node2_1/topic_500K_6',
        '/node2_2/topic_1M_1',
        '/node2_2/topic_1M_2',
        '/node2_2/topic_1M_3',
        '/node2_2/topic_1M_4',
        '/node2_2/topic_2M_1',
        '/node2_2/topic_2M_2',
        '/node2_2/topic_2M_3',
        '/node2_2/topic_2M_4',
        '/node2_2/topic_2M_5',
        '/node2_2/topic_500K_1',
        '/node2_2/topic_500K_2',
        '/node2_2/topic_500K_3',
        '/node2_2/topic_500K_4',
        '/node2_2/topic_500K_5',
        '/node2_2/topic_500K_6'
    ]
    launch_str = 'ros2 launch fabric_nodes fabric_nodes.launch.py'
    topic_list_str = 'ros2 topic list'

    # Start the launch subprocess
    launch_process = subprocess.Popen(launch_str, shell=True, preexec_fn=os.setpgrp)
    time.sleep(10)

    # Get topic list
    topic_list_process = subprocess.run(topic_list_str, shell=True, capture_output=True, text=True)
    topic_list = topic_list_process.stdout.strip().splitlines()

    # Terminate the launch subprocess
    os.killpg(os.getpgid(launch_process.pid), signal.SIGINT)
    launch_process.wait()

    # Check topic list
    for expect_topic in expect_topics:
        assert expect_topic in topic_list, f"'expect_node '{expect_topic}' not processed"

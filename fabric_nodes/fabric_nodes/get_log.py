#!/usr/bin/env python3
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


import rclpy
import re
import pandas as pd

from rclpy.node import Node

class GetLog(Node):
    """
    Gather statistics from log files for a given period or run

    Parameters
    ----------
        time (int): Logging duration in nanoseconds
        run_id (str): ID for a specific run

    """

    def __init__(self, time = 0, run_id = "default_run"):
        super().__init__('get_log')
        self.time = time
        self.run_id = run_id

    def read_log(self):
        self.lines = open("log.txt", "r").readlines()
        for line in self.lines:
            stats_log = re.search(r"\[node.*:\sT.*", line)
            if stats_log is not None:  # valid lines
                print(stats_log.group())
    def output_log(self):
        """Outputing the parsed statistics"""
        self.get_logger().info(str(self.time) + " seconds on " + self.run_id)


def main(args=None):
    rclpy.init(args=args)
    m_log = GetLog(5)
    m_log.read_log()
    m_log.output_log()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
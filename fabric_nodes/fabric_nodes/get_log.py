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
        lines = open("log.txt", "r").readlines()
        parsed_log = []
        for line in lines:
            stats_log = re.search(r"\[node.*:\sT.*", line)
            if stats_log is not None:  # valid lines
                topic_name = str(re.search(r"(?<=Topic:\s).*,", stats_log.group()).group()) or None
                sub_node = str(re.search(r"(?<=Topic:\s/).*(?=/)", stats_log.group()).group()) or None
                pub_node = str(re.search(r"(?<=\[).*(?=\.)", stats_log.group()).group()) or None
                ros_sub_stamp = str(re.search(r"(?<=ROS\sxmt\stime\sns:\s).*", stats_log.group()).group()) or None
                ros_pub_stamp = None
                rmw_sub_stamp = None
                rmw_pub_stamp = None
                parsed_log.append([topic_name, sub_node, pub_node, 
                                  ros_sub_stamp, ros_pub_stamp,
                                  rmw_sub_stamp, rmw_pub_stamp])
                # print(stats_log.group())
        self.parsed_log_df = pd.DataFrame(parsed_log, columns=['Topic', 'Subscriber Node', 'Publisher Node',
                                               'ROS Layer Subscriber Timestamp', 'ROS Layer Publisher Timestamp',
                                               'RMW Layer Subscriber Timestamp', 'RMW Layer Publisher Timestamp'])

    def output_log(self):
        """Outputing the parsed statistics"""
        self.parsed_log_df.to_csv("log.csv", sep='\t', index=False)
        self.get_logger().info(str(self.time) + " seconds on " + self.run_id)


def main(args=None):
    rclpy.init(args=args)
    m_log = GetLog(5)
    m_log.read_log()
    m_log.output_log()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
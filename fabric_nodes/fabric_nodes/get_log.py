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


import os
import re

import pandas as pd
import rclpy

from rclpy.node import Node


class GetLog(Node):
    """
    Gather statistics from log files for a given period or run.

    Parameters
    ----------
        time (int): Logging duration in nanoseconds
        run_id (str): ID for a specific run

    """

    def __init__(self, time=0, run_id='default_run'):
        super().__init__('get_log')
        self.time = time
        self.run_id = run_id

    def read_log(self):
        ros_log_dir = os.path.expanduser('~') + '/.ros/log/'
        dirlist = list(d for d in next(os.walk(ros_log_dir))[1])
        dirlist.sort()
        if (self.run_id == 'default_run'):
            self.run_id = dirlist[-1]
        logfile_path = ros_log_dir + self.run_id + '/launch.log'
        self.lines = open(logfile_path, 'r').readlines()
        self.get_logger().info('Reading log from ' + logfile_path)

    def parse_log(self):
        begin_timestamp = float(re.search(r'\d*\.\d*', self.lines[0]).group())
        current_timestamp = begin_timestamp
        use_input_time = False

        parsed_log = []
        for line in self.lines:
            stats_log = re.search(r'\[node.*:\sT.*', line)
            if stats_log is not None:  # valid lines
                current_timestamp = float(re.search(r'\d*\.\d*(?=\s)', line).group())
                if (current_timestamp > begin_timestamp + self.time):
                    self.get_logger().info('Starting at ' + str(begin_timestamp) +
                                           ', ending at ' + str(current_timestamp))
                    use_input_time = True
                    break
                topic_name = str(re.search(r'(?<=Topic:\s).*(?=,)',
                                           stats_log.group()).group()) or None
                sub_node = str(re.search(r'(?<=Topic:\s/).*(?=/)',
                                         stats_log.group()).group()) or None
                pub_node = str(re.search(r'(?<=\[).*(?=\.)',
                                         stats_log.group()).group()) or None
                ros_sub_time = str(re.search(r'(?<=ROS\sxmt\stime\sns:\s).*',
                                             stats_log.group()).group()) or None
                ros_pub_time = None
                rmw_sub_time = None
                rmw_pub_time = None
                parsed_log.append([topic_name, sub_node, pub_node,
                                  ros_sub_time, ros_pub_time,
                                  rmw_sub_time, rmw_pub_time])
        if (not use_input_time):
            self.get_logger().info('This given log has less than ' + str(self.time) + ' seconds.')
            self.time = current_timestamp - begin_timestamp
        self.parsed_log_df = pd.DataFrame(parsed_log, columns=[
                                            'Topic', 'Subscriber Node', 'Publisher Node',
                                            'ROS Layer Subscriber Time',
                                            'ROS Layer Publisher Time',
                                            'RMW Layer Subscriber Time',
                                            'RMW Layer Publisher Time'])

    def output_log(self):
        """Output the parsed statistics."""
        self.parsed_log_df.to_csv('log.csv', sep='\t', index=False)
        self.get_logger().info(str(self.time) + ' seconds on run: ' + self.run_id)


def main(args=None):
    rclpy.init(args=args)
    m_log = GetLog(5)
    m_log.read_log()
    m_log.parse_log()
    m_log.output_log()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

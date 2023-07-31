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

import numpy as np
import matplotlib.pyplot as plt
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

    def search_ros_log(self, log):
        topic_name = str(re.search(r'(?<=Topic:\s).*(?=,\sROS\sxmt)', log).group()) or None
        sub_node = str(re.search(r'(?<=\[)node.*(?=\.n)', log).group()) or None
        pub_node = str(re.search(r'(?<=Topic:\s\/).*(?=\/)', log).group()) or None
        ros_time = str(re.search(r'(?<=ROS\sxmt\stime\sns:\s)\d*', log).group()) or None
        ros_sub_time = str(re.search(r'(?<=ROSSUB\sTS:\s)\d*', log).group()) or None
        ros_pub_time = str(re.search(r'(?<=ROSPUB\sTS:\s)\d*', log).group()) or None
        return [topic_name, sub_node, pub_node,
                ros_time, ros_sub_time, ros_pub_time]

    def search_rmw_log(self, log):
        rmw_time = str(re.search(r'(?<=rmw\sxmt\stime\sns:\s)\d*', log).group()) or None
        rmw_sub_time = str(re.search(r'(?<=RMWSUB\sTS:\s)\d*', log).group()) or None
        rmw_pub_time = str(re.search(r'(?<=RMWPUB\sTS:\s)\d*', log).group()) or None
        return [rmw_time, rmw_sub_time, rmw_pub_time]

    def parse_log(self):
        begin_timestamp = float(re.search(r'\d*\.\d*', self.lines[0]).group())
        current_timestamp = begin_timestamp
        use_input_time = False

        parsed_rmw = []
        parsed_log = []
        for line in self.lines:
            current_timestamp = float(re.search(r'\d*\.\d*(?=\s)', line).group())
            if (current_timestamp > begin_timestamp + self.time):
                self.get_logger().info('Starting at ' + str(begin_timestamp) +
                                        ', ending at ' + str(current_timestamp))
                use_input_time = True
                break
            stats_log = re.search(r'\[node.*:\sT.*', line)
            if stats_log is not None:  # valid lines
                rmw_log = re.search(r'\[node.*.rmw]:\sT.*', line)
                if rmw_log is not None:
                    parsed_rmw = self.search_rmw_log(rmw_log.group())
                    continue
                parsed_log.append(self.search_ros_log(stats_log.group())+parsed_rmw)
        if (not use_input_time):
            self.get_logger().info('This given log has less than ' + str(self.time) + ' seconds.')
            self.time = current_timestamp - begin_timestamp
        self.parsed_log_df = pd.DataFrame(parsed_log, columns=[
                                            'Topic', 'Subscriber Node', 'Publisher Node',
                                            'ROS Layer Transmission Time',
                                            'ROS Layer Subscriber Time',
                                            'ROS Layer Publisher Time',
                                            'RMW Layer Transmission Time',
                                            'RMW Layer Subscriber Time',
                                            'RMW Layer Publisher Time'])

    def output_log(self):
        """Output the parsed statistics."""
        self.parsed_log_df.to_csv('log.csv', sep='\t', index=False)
        self.get_logger().info(str(self.time) + ' seconds on run: ' + self.run_id)
        self.ros_xmt_time = list(map(int, self.parsed_log_df['ROS Layer Transmission Time']))
        self.get_logger().info(
            'Average ROS XMT is: ' + str(np.mean(self.ros_xmt_time)) + ' ns, ' +
            'with a standard deviation of ' + str(np.std(self.ros_xmt_time)) + ' ns.')

    def plot_log(self):
        plt.hist(self.ros_xmt_time, color='blue', edgecolor='black')
        plt.title('ROS Layer Transmission Time')
        plt.xlabel('Time (Nanoseconds)')
        plt.ylabel('Occurrences')
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    m_log = GetLog(5)
    m_log.read_log()
    m_log.parse_log()
    m_log.output_log()
    m_log.plot_log()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

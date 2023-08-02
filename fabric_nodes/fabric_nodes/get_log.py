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
        self.parsed_log_df = self.parsed_log_df.astype({"ROS Layer Transmission Time": "int",
                                                        "ROS Layer Subscriber Time": "int",
                                                        "ROS Layer Publisher Time": "int",
                                                        "RMW Layer Transmission Time": "int",
                                                        "RMW Layer Subscriber Time": "int",
                                                        "RMW Layer Publisher Time": "int"})

    def output_log(self):
        """Output the parsed statistics."""
        self.parsed_log_df.to_csv((
            str(round(self.time, 3))+'-seconds-'+self.run_id+'.csv'), sep='\t', index=False)
        self.get_logger().info(str(self.time) + ' seconds on run: ' + self.run_id)

        self.ros_xmt_time = list(self.parsed_log_df['ROS Layer Transmission Time'])
        self.rmw_xmt_time = list(self.parsed_log_df['RMW Layer Transmission Time'])

        self.parsed_df_by_topics = self.parsed_log_df.groupby('Topic').agg(
            avg_ros_time=('ROS Layer Transmission Time', np.mean),
            avg_rmw_time=('RMW Layer Transmission Time', np.mean)
        ).reset_index()

        self.get_logger().info(
            'Average ROS XMT is: ' + str(np.mean(self.ros_xmt_time)) + ' ns, ' +
            'with a standard deviation of ' + str(np.std(self.ros_xmt_time)) + ' ns.')
        self.get_logger().info(
            'Average RMW XMT is: ' + str(np.mean(self.rmw_xmt_time)) + ' ns, ' +
            'with a standard deviation of ' + str(np.std(self.rmw_xmt_time)) + ' ns.')

    def plot_log(self):
        self.plot_hist_xmt_time()
        # self.plot_bar_ros_xmt_by_topics()
        # self.plot_bar_rmw_xmt_by_topics()
        # self.plot_log_diff_xmt_by_topics()
        plt.show()

    def plot_hist_xmt_time(self):
        ax1 = plt.subplot(1, 2, 1)
        ax1.hist(self.ros_xmt_time, color='blue', edgecolor='black')
        ax1.set_title('ROS Layer Transmission Time')
        ax1.set_xlabel('Time (Nanoseconds)')
        ax1.set_ylabel('Occurrences')

        ax2 = plt.subplot(1, 2, 2)
        ax2.hist(self.rmw_xmt_time, color='yellow', edgecolor='black')
        ax2.set_title('RMW Layer Transmission Time')
        ax2.set_xlabel('Time (Nanoseconds)')
        ax2.set_ylabel('Occurrences')

    def plot_bar_ros_xmt_by_topics(self):
        plt.bar(list(self.parsed_df_by_topics['Topic']),
                list(self.parsed_df_by_topics['avg_ros_time']))
        plt.title('ROS Layer Transmission Time By Topics')
        plt.xlabel('Topic Name')
        plt.ylabel('Time (Nanoseconds)')
        plt.xticks(rotation=90, fontsize=6)

    def plot_bar_rmw_xmt_by_topics(self):
        plt.bar(list(self.parsed_df_by_topics['Topic']),
                list(self.parsed_df_by_topics['avg_rmw_time']))
        plt.title('RMW Layer Transmission Time By Topics')
        plt.xlabel('Topic Name')
        plt.ylabel('Time (Nanoseconds)')
        plt.xticks(rotation=90, fontsize=6)

    def plot_log_diff_xmt_by_topics(self):
        plt.plot(range(len(list(self.parsed_df_by_topics['Topic']))),
                 np.divide(list(self.parsed_df_by_topics['avg_ros_time']),
                           list(self.parsed_df_by_topics['avg_rmw_time'])))
        plt.grid()
        plt.title('Log of (ROS_TIME/RMW_TIME) by Topics')
        plt.xlabel('Topic Name')
        plt.ylabel('Log value')
        plt.xticks(ticks=range(len(list(self.parsed_df_by_topics['Topic']))),
                   labels=list(self.parsed_df_by_topics['Topic']),
                   rotation=90, fontsize=6)


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

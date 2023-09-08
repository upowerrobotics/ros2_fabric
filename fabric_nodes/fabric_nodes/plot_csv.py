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
import sys, getopt

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rclpy

from rclpy.node import Node


##
# @class PlotCSV.
# @brief Plotting based on statistics from csv files.
#
class PlotCSV(Node):

    ##
    # @brief The contructor for PlotCSV.
    # @param [in] dds Data Distribution Service middleware.
    # @param [in] input_csv_file Input csv name.
    #
    def __init__(self, dds, input_csv_file):
        super().__init__('plot_csv')
        self.dds = dds
        self.input_csv_file = input_csv_file

    ##
    # @brief Read either the latest csv generated or specific input file
    #
    def read_csv(self):
        if self.input_csv_file == 'latest':
            files = [file for file in os.listdir('.') if (file.lower().endswith('time_log.csv'))]
            files.sort(key=os.path.getmtime)
            self.input_csv_file = files[-1]
        self.parsed_log_df = pd.read_csv(self.input_csv_file, sep='\t')
        self.ros_xmt_time = list(self.parsed_log_df['ROS Layer Transmission Time'])
        self.rmw_xmt_time = list(self.parsed_log_df['RMW Layer Transmission Time'])
        self.parsed_df_by_topics = self.parsed_log_df.groupby('Topic').agg(
            avg_ros_time=('ROS Layer Transmission Time', np.mean),
            avg_rmw_time=('RMW Layer Transmission Time', np.mean)
        ).reset_index()
        self.each_topic_parsed_log_df = []
        topics_groups = self.parsed_log_df.sort_values([
            'Topic', 'ROS Layer Publisher Time'], ascending=True).groupby('Topic')
        for gp in topics_groups.groups:
            self.each_topic_parsed_log_df.append(topics_groups.get_group(gp))

    ##
    # @brief Generate various plots to visualize the log data.
    #
    def plot_csv(self):
        # self.plot_bar_xmt_by_topics()
        # self.plot_diff_xmt_by_topics()
        self.plot_topic_time_series(self.each_topic_parsed_log_df[0])
        plt.show()

    ##
    # @brief Plot histograms for transmission times.
    #
    def plot_hist_xmt_time(self):
        ax1 = plt.subplot(1, 2, 1)
        ax1.hist(self.ros_xmt_time, color='blue', edgecolor='black')
        ax1.set_title(self.dds + ' ROS Layer Transmission Time')
        ax1.set_xlabel('Time (Nanoseconds)')
        ax1.set_ylabel('Occurrences')

        ax2 = plt.subplot(1, 2, 2)
        ax2.hist(self.rmw_xmt_time, color='yellow', edgecolor='black')
        ax2.set_title(self.dds + ' RMW Layer Transmission Time')
        ax2.set_xlabel('Time (Nanoseconds)')
        ax2.set_ylabel('Occurrences')

    ##
    # @brief Plot average transmission time by topics.
    #
    def plot_bar_xmt_by_topics(self):
        plt.bar(list(self.parsed_df_by_topics['Topic']),
                list(self.parsed_df_by_topics['avg_ros_time']))
        plt.bar(list(self.parsed_df_by_topics['Topic']),
                list(self.parsed_df_by_topics['avg_rmw_time']))
        plt.legend(['Average ROS Transmission Duration', 'Average RMW Transmission Duration'])
        plt.title(self.dds + ' Average Transmission Time By Topics')
        plt.xlabel('Topic Name')
        plt.ylabel('Time (Nanoseconds)')
        plt.xticks(rotation=90, fontsize=6)

    ##
    # @brief Plot difference in transmission time by topics.
    #
    def plot_diff_xmt_by_topics(self):
        plt.plot(range(len(list(self.parsed_df_by_topics['Topic']))),
                 np.subtract(list(self.parsed_df_by_topics['avg_ros_time']),
                             list(self.parsed_df_by_topics['avg_rmw_time'])))
        plt.grid()
        plt.title(self.dds + ' Average (ROS-RMW) Time by Topics')
        plt.xlabel('Topic Name')
        plt.ylabel('Time Difference (Nanoseconds)')
        plt.xticks(ticks=range(len(list(self.parsed_df_by_topics['Topic']))),
                   labels=list(self.parsed_df_by_topics['Topic']),
                   rotation=90, fontsize=6)

    ##
    # @brief Plot time series for each topic.
    # @param topic_df DataFrame containing the log data for a specific topic
    #
    def plot_topic_time_series(self, topic_df):
        ax1 = plt.subplot(2, 1, 1)
        ax1.plot(topic_df['ROS Layer Publisher Time'], topic_df['ROS Layer Transmission Time'])
        ax1.plot(topic_df['ROS Layer Publisher Time'], topic_df['RMW Layer Transmission Time'])
        ax1.legend(['ROS Transmission Duration', 'RMW Transmission Duration'])
        ax1.grid()
        ax1.set_title(self.dds + ' Transmission Time Series for ' + list(topic_df['Topic'])[0])
        ax1.set_xlabel('Timestamps')
        ax1.set_ylabel('Transmission Time Duration (Nanoseconds)')
        ax1.set_xticks([])

        ax2 = plt.subplot(2, 1, 2)
        ax2.plot(topic_df['ROS Layer Publisher Time'],
                 topic_df['ROS Layer Transmission Time'] -
                 topic_df['RMW Layer Transmission Time'], 'g')
        ax2.grid()
        ax2.set_title(self.dds +
                      ' Difference in Transmission Time Series for ' + list(topic_df['Topic'])[0])
        ax2.set_xlabel('Timestamps')
        ax2.set_ylabel('(ROS-RMW) Transmission Time Duration (Nanoseconds)')
        ax2.set_xticks([])


##
# @brief Retrieve Argument from Command Line. Initialize the ROS node and run the PlotCSV class methods.
#
def main(argv):
    rclpy.init(args=None)
    inputfile = "latest"
    opts, _ = getopt.getopt(argv,"hi:",["input_csv="])
    for opt, arg in opts:
        if opt == '-h':
            print ('Correct Usage: ros2 run fabric_nodes plot_csv.py -i/--input_csv <inputfile>')
            sys.exit()
        elif opt in ("-i", "--input_csv"):
            inputfile = arg
    m_plot = PlotCSV(dds='rmw_cyclonedds', input_csv_file=inputfile)
    m_plot.read_csv()
    m_plot.plot_csv()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])

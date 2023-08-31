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

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rclpy

from rclpy.node import Node


##
# @class GetLog.
# @brief Gather statistics from log files for a given period or run.
#
class GetLog(Node):

    ##
    # @brief The contructor for GetLog.
    # @param [in] time Logging duration in nanoseconds.
    # @param [in] dds Data Distribution Service middleware.
    # @param [in] run_id ID for a specific run.
    #
    def __init__(self, time=0, dds='rmw_cyclonedds', run_id='default_run'):
        super().__init__('get_log')
        ## Logging duration in nanoseconds.
        self.time = time
        ## Data Distribution Service middleware that was operated.
        self.dds = dds
        ## ID for a specific run.
        self.run_id = run_id

    ##
    # @brief Read log file and load its lines into self.lines.
    #
    def read_log(self):
        ros_log_dir = os.path.join(os.path.expanduser('~') + '/.ros/log/')
        dirlist = list(next(os.walk(ros_log_dir))[1])
        dirlist.sort()
        if (self.run_id == 'default_run'):
            self.run_id = dirlist[-1]
        logfile_path = os.path.join(ros_log_dir, self.run_id, 'launch.log')
        ## The line that extracted from log.
        self.lines = open(logfile_path, 'r').readlines()
        self.get_logger().info('Reading log from ' + logfile_path)

    ##
    # @brief Parse ROS logs and extract various information.
    # @param [in] log A line from the log file.
    # @return The extracted ros log data as a list.
    #
    def search_ros_log(self, log):
        topic_name = str(re.search(r'(?<=Topic:\s).*(?=,\sROS\sxmt)', log).group()) or None
        sub_node = str(re.search(r'(?<=\[)(.*node.*)(?=.\1\])', log).group()) or None
        pub_node = str(re.search(r'(?<=Topic:\s\/).*(?=\/)', log).group()) or None
        ros_time = str(re.search(r'(?<=ROS\sxmt\stime\sns:\s)\d*', log).group()) or None
        ros_sub_time = str(re.search(r'(?<=ROSSUB\sTS:\s)\d*', log).group()) or None
        ros_pub_time = str(re.search(r'(?<=ROSPUB\sTS:\s)\d*', log).group()) or None
        drop_num = str(re.search(r'(?<=Drop\sNum:\s)\d*', log).group()) or None
        receive_rate = str(re.search(r'(?<=Recieve\sRate:\s)\d*.\d*', log).group()) or None
        return [topic_name, sub_node, pub_node,
                ros_time, ros_sub_time, ros_pub_time, drop_num, receive_rate]

    ##
    # @brief Parse RMW logs and extract various information.
    # @param [in] log A line from the log file.
    # @return The extracted rmw log data as a list.
    #
    def search_rmw_log(self, log):
        rmw_time = str(re.search(r'(?<=rmw\sxmt\stime\sns:\s)\d*', log).group()) or None
        rmw_sub_time = str(re.search(r'(?<=RMWSUB\sTS:\s)\d*', log).group()) or None
        rmw_pub_time = str(re.search(r'(?<=RMWPUB\sTS:\s)\d*', log).group()) or None
        return [rmw_time, rmw_sub_time, rmw_pub_time]

    ##
    # @brief Parse frequency and bandwidth logs and extract various information.
    # @param [in] log A line from the log file.
    # @return The extracted frequency and bandwidth log data as a list.
    #
    def search_freq_bw_log(self, log):
        time_stamp = str(re.search(r'(?<=\[)\d*\.\d*(?=\])', log).group()) or None
        topic_name = str(re.search(r'(?<=Topic:\s).*(?=,\sF)', log).group()) or None
        topic_freq = str(re.search(r'(?<=Freq:\s)\d*\.\d*', log).group()) or None
        topic_bw = str(re.search(r'(?<=Bandwidth:\s)\d*', log).group()) or None
        return [time_stamp, topic_name, topic_freq, topic_bw]

    ##
    # @brief Parse the entire log file to gather statistics.
    #
    def parse_log(self):
        begin_timestamp = float(re.search(r'\d*\.\d*', self.lines[0]).group())
        current_timestamp = begin_timestamp
        use_input_time = False

        parsed_rmw = []
        parsed_log = []
        parsed_freq_bw = []
        for line in self.lines:
            current_timestamp = float(re.search(r'\d*\.\d*(?=\s)', line).group())
            if (current_timestamp > begin_timestamp + self.time):
                self.get_logger().info('Starting at ' + str(begin_timestamp) +
                                       ', ending at ' + str(current_timestamp))
                use_input_time = True
                break
            stats_log = re.search(r'(?<=\]\s)\[.*node.*:\sT.*', line)
            if stats_log is not None:  # valid lines
                rmw_log = re.search(r'\[.*node.*\.(C|F|e|R).*]:\sT.*', line)
                freq_bw_log = re.search(r'Freq:', stats_log.group())
                if rmw_log is not None:
                    parsed_rmw = self.search_rmw_log(rmw_log.group())
                    continue
                if freq_bw_log is not None:
                    parsed_freq_bw.append(self.search_freq_bw_log(line))
                    continue
                parsed_log.append(self.search_ros_log(stats_log.group())+parsed_rmw)
        if (not use_input_time):
            self.get_logger().info('This given log has less than ' + str(self.time) + ' seconds.')
            ## The time difference between the logger.
            self.time = current_timestamp - begin_timestamp
        ## The parsed Sub Time, Pub Time, and Trans Time in pd.DataFrame format.
        self.parsed_log_df = pd.DataFrame(parsed_log, columns=[
                                            'Topic', 'Subscriber Node', 'Publisher Node',
                                            'ROS Layer Transmission Time',
                                            'ROS Layer Subscriber Time',
                                            'ROS Layer Publisher Time',
                                            'ROS Layer Number of Dropped Messages',
                                            'ROS Layer Accumulative Receive Rate',
                                            'RMW Layer Transmission Time',
                                            'RMW Layer Subscriber Time',
                                            'RMW Layer Publisher Time'])
        self.parsed_log_df = self.parsed_log_df.astype({'ROS Layer Transmission Time': 'int',
                                                        'RMW Layer Transmission Time': 'int'})
        ## The parsed measured frequency and bandwidth in pd.DataFrame format.
        self.parsed_freq_bw_df = pd.DataFrame(parsed_freq_bw, columns=[
                                            'Timestamp', 'Topic', 'Frequency', 'Bandwidth'])

    ##
    # @brief Output the parsed statistics.
    #
    def output_log(self):
        outlier_indices_neg = self.parsed_log_df[(
                              self.parsed_log_df['ROS Layer Transmission Time'] <
                              self.parsed_log_df['RMW Layer Transmission Time'])].index
        outlier_indices_large = self.parsed_log_df[(
                                self.parsed_log_df['ROS Layer Transmission Time'] -
                                self.parsed_log_df['RMW Layer Transmission Time'] > 50000)].index
        self.parsed_log_df.drop(outlier_indices_neg.union(outlier_indices_large), inplace=True)
        self.parsed_log_df.to_csv((
            str(round(self.time, 3)) + '-seconds-' +
            self.run_id+'_time_log.csv'), sep='\t', index=False)
        self.parsed_freq_bw_df.to_csv((
            str(round(self.time, 3)) + '-seconds-' +
            self.run_id+'_fequency_bw_log.csv'), sep='\t', index=False)
        self.get_logger().info(str(self.time) +
                               ' seconds on run: ' + self.run_id + ' with ' + self.dds)

        ## The xmt time of the ros layer.
        self.ros_xmt_time = list(self.parsed_log_df['ROS Layer Transmission Time'])
        ## The xmt time of the rmw layer.
        self.rmw_xmt_time = list(self.parsed_log_df['RMW Layer Transmission Time'])

        ## The parsed pd.DataFrame format based on topics.
        self.parsed_df_by_topics = self.parsed_log_df.groupby('Topic').agg(
            avg_ros_time=('ROS Layer Transmission Time', np.mean),
            avg_rmw_time=('RMW Layer Transmission Time', np.mean)
        ).reset_index()

        ## The parsed pd.DataFrame format of each topics.
        self.each_topic_parsed_log_df = []
        topics_groups = self.parsed_log_df.sort_values([
            'Topic', 'ROS Layer Publisher Time'], ascending=True).groupby('Topic')
        for gp in topics_groups.groups:
            self.each_topic_parsed_log_df.append(topics_groups.get_group(gp))

        self.get_logger().info(
            'Average ROS XMT is: ' + str(np.mean(self.ros_xmt_time)) + ' ns, ' +
            'with a standard deviation of ' + str(np.std(self.ros_xmt_time)) + ' ns.')
        self.get_logger().info(
            'Average RMW XMT is: ' + str(np.mean(self.rmw_xmt_time)) + ' ns, ' +
            'with a standard deviation of ' + str(np.std(self.rmw_xmt_time)) + ' ns.')

    ##
    # @brief Generate various plots to visualize the log data.
    #
    def plot_log(self):
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
# @brief Initialize the ROS node and run the GetLog class methods.
# @param Command-line Command-line arguments.
#
def main(args=None):
    rclpy.init(args=args)
    m_log = GetLog(60, 'rmw_cyclonedds')
    m_log.read_log()
    m_log.parse_log()
    m_log.output_log()
    m_log.plot_log()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

// Copyright 2023 U Power Robotics USA, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file dummy_node.hpp
 * @brief Header file for the DummyNode class.
 *        This file contains the declaration of the DummyNode class.
 *        It also defines related structures and enumerations.
 * @author U Power Robotics USA Inc
 * @date September 7, 2023
 * @license Apache License, Version 2.0
 *          http://www.apache.org/licenses/LICENSE-2.0
 */

#ifndef FABRIC_NODES__DUMMY_NODE_HPP_
#define FABRIC_NODES__DUMMY_NODE_HPP_

#include <string>
#include <vector>

// The NOLINTs below are to fix Galactic bugs
#include <rclcpp/rclcpp.hpp>  // NOLINT
#include <fabric_interfaces/msg/dummy_message.hpp>  // NOLINT

using DummyMsgT = fabric_interfaces::msg::DummyMessage;

namespace fabric_nodes
{

/**
 * @brief Enumeration for different size types used for bandwidth and message size.
 */
enum class SizeType
{
  BYTES = 1LL,  ///< Bytes size type.
  KILOBYTES = 1024LL,  ///< Kilobytes size type.
  MEGABYTES = 1048576LL,  ///< Megabytes size type.
  GIGABYTES = 1073741824LL  ///< Gigabytes size type.
};

/**
 * @struct PublishTopic
 * @brief Structure to store information about a topic to be published.
 */
struct PublishTopic
{
  float bandwidth_scalar = 0.0f;  ///< Bandwidth scalar value.
  SizeType bandwidth_size_type = SizeType::BYTES;  ///< Bandwidth size type.
  float frequency = 0.0f;  ///< Publishing frequency.
  float msg_size_scalar = 0.0f;  ///< Message size scalar value.
  SizeType msg_size_type = SizeType::BYTES;  ///< Message size type.
  std::string topic_name = "";  ///< Topic name to publish.
  rclcpp::Publisher<DummyMsgT>::SharedPtr publisher;  ///< Publisher for the topic.
  rclcpp::TimerBase::SharedPtr publish_timer;  ///< Timer for periodic publishing.
  int64_t seq_num = 0;  ///< Sequence number for the published messages.
};

/**
 * @struct SubscribeTopic
 * @brief Structure to store information about a topic to be subscribed to.
 */
struct SubscribeTopic
{
  std::string node_name = "";  ///< Name of the node subscribing to the topic.
  std::string topic_name = "";  ///< Topic name to subscribe.
  rclcpp::Subscription<DummyMsgT>::SharedPtr subscriber;  ///< Subscriber for the topic.
  int64_t seq_num = 0;  ///< Sequence number for the received messages.
  int64_t drop_msg_num = 0;  ///< Number of dropped messages.
  int64_t receive_num = 0;  ///< Number of received messages.
  rclcpp::Time initial_freq_time;  ///< Time when frequency calculation started.
  size_t revieve_bytes = 0;  ///< Number of received bytes.
};

/**
 * @class DummyNode
 * @brief A class representing the DummyNode.
 *        This class handles publishing and subscribing to topics and performing other operations.
 */
class DummyNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the DummyNode class.
   * @param options Node options.
   */
  explicit DummyNode(rclcpp::NodeOptions options);

private:
  /**
   * @brief Parse and handle the parameters related to publishing topics.
   * @param param_prefix The prefix for the publish topics' parameters.
   */
  void parse_publish_topic(const std::string & param_prefix);

  /**
   * @brief Parse and handle the parameters related to subscribing topics.
   * @param subscribe_prefix The prefix for the subscribe topics' parameters.
   */
  void parse_subscribe_topic(const std::string & subscribe_prefix);

  /**
   * @brief Parse the data size specified as a string and extract the scalar and type.
   * @param data_size The data size as a string (e.g., "1KB").
   * @param scalar Output parameter for the extracted scalar value.
   * @param type Output parameter for the extracted size type.
   * @return True if parsing is successful, false otherwise.
   */
  bool parse_data_size(const std::string & data_size, float * scalar, SizeType * type);

  /**
   * @brief Callback function for publishing messages.
   * @param publisher The publisher used to publish the message.
   * @param msg_bytes The size of the message in bytes.
   * @param seq_num Reference to the sequence number for the published messages.
   */
  void pub_callback(
    rclcpp::Publisher<DummyMsgT>::SharedPtr publisher, uint64_t msg_bytes,
    int64_t & seq_num);

  /**
   * @brief Callback function for receiving subscribed messages.
   * @param msg The received message.
   * @param topic_name The name of the topic from which the message was received.
   * @param seq_num Reference to the sequence number for the received messages.
   * @param drop_msg_num Reference to the number of dropped messages.
   * @param receive_num Reference to the number of received messages.
   * @param initial_freq_time Reference to the time when frequency calculation started.
   * @param revieve_bytes Reference to the number of received bytes.
   */
  void sub_callback(
    const DummyMsgT::SharedPtr msg, const std::string & topic_name,
    int64_t & seq_num, int64_t & drop_msg_num, int64_t & receive_num,
    rclcpp::Time & initial_freq_time, size_t & revieve_bytes);

  /**
   * @brief Format the byte value into human-readable format (e.g., KB, MB, GB).
   * @param byte The byte value to format.
   * @return The formatted string representing the byte value.
   */
  std::string bw_format(const size_t byte);

  bool m_root_node = false;  ///< Flag indicating if this is a root node.
  bool m_terminal_node = false;  ///< Flag indicating if this is a terminal node.
  std::vector<PublishTopic> m_publish_topics;  ///< Vector storing information about
                                               ///< publish topics.
  std::vector<SubscribeTopic> m_subscribe_topics;  ///< Vector storing information about
                                                   ///< subscribe topics.
};

}  // namespace fabric_nodes

#endif  // FABRIC_NODES__DUMMY_NODE_HPP_

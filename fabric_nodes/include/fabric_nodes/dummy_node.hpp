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

#ifndef FABRIC_NODES__DUMMY_NODE_HPP_
#define FABRIC_NODES__DUMMY_NODE_HPP_

#include <string>
#include <vector>

// The NOLINTs below are to fix Galactic bugs
#include <rclcpp/rclcpp.hpp>  // NOLINT
#include <fabric_interfaces/msg/dummy_message.hpp>  // NOLINT

using DummyMsgT = fabric_interfaces::msg::DummyMessage;  ///< The message type that dummynode is

/**
 * @brief Calculates the length of a null-terminated C-style string.
 * @param [in] str A pointer to the null-terminated C-style string.
 * @return The length of the string (number of characters) excluding the null terminator.
 */
int constexpr char_len(const char * str);

/**
 * @brief Namespace containing classes and functions for the fabric nodes.
 */
namespace fabric_nodes
{

/**
 * @brief Enum for specifying size types in Bytes, Kilobytes, Megabytes, or Gigabytes.
 */
enum class SizeType
{
  BYTES = 1LL,
  KILOBYTES = 1024LL,
  MEGABYTES = 1048576LL,
  GIGABYTES = 1073741824LL
};

/**
 * @brief Struct to hold the settings of QoS (history depth and policy).
 */
struct QosSetting
{
  size_t depth = 1;  ///< Set the history depth to keep last
  std::string policy = "reliable";  ///< Set the reliability setting to 'reliable' or 'best_effort'
};

/**
 * @brief Struct to hold all information related to a Publish topic.
 */
struct PublishTopic
{
  QosSetting qos_setting;
  float bandwidth_scalar = 0.0f;  ///< Bandwidth scalar
  SizeType bandwidth_size_type = SizeType::BYTES;  ///< Size type for bandwidth
  float frequency = 0.0f;  ///< Publishing frequency
  float msg_size_scalar = 0.0f;  ///< Message size scalar
  SizeType msg_size_type = SizeType::BYTES;  ///< Size type for message size
  std::string topic_name = "";  ///< Name of the topic
  rclcpp::Publisher<DummyMsgT>::SharedPtr publisher;  ///< ROS2 Publisher
  rclcpp::TimerBase::SharedPtr publish_timer;  ///< Timer for publishing
  int64_t seq_num = 0;  ///< Sequence number
};

/**
 * @brief Struct to hold all information related to a Subscribe topic.
 */
struct SubscribeTopic
{
  QosSetting qos_setting;
  std::string node_name = "";  ///< Node name
  std::string topic_name = "";  ///< Name of the topic
  rclcpp::Subscription<DummyMsgT>::SharedPtr subscriber;  ///< ROS2 Subscriber
  int64_t seq_num = 0;  ///< Sequence number
  int64_t drop_msg_num = 0;  ///< Dropped message count
  int64_t receive_num = 0;  ///< Received message count
  rclcpp::Time initial_freq_time;  ///< Initial frequency time for topic
  size_t revieve_bytes = 0;  ///< Received bytes for topic
};

/**
 * @class DummyNode
 * @brief A ROS2 Node for dummy publish and subscribe functionalities.
 */
class DummyNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructs a DummyNode for ROS2
   *
   * The constructor for the DummyNode, which inherits from rclcpp::Node.
   * It sets up publishers and subscribers based on parameters.
   *
   * @param [in] options ROS2 Node options.
   * @exception rclcpp::exceptions::InvalidParametersException Throws an exception if required parameters are missing.
   */
  explicit DummyNode(rclcpp::NodeOptions options);

private:
  /**
   * @brief Parses and sets up a publish topic
   *
   * This function parses parameters to setup a topic that this node will publish to.
   *
   * @param [in] param_prefix The prefix string for parameters related to this topic.
   * @exception rclcpp::exceptions::InvalidParameterValueException Throws if parameter values are incorrect.
   */
  void parse_publish_topic(const std::string & param_prefix);

  /**
   * @brief Parses and sets up a subscribe topic
   *
   * This function parses parameters to setup a topic to which this node will subscribe.
   *
   * @param [in] subscribe_prefix The prefix string for parameters related to this topic.
   * @exception rclcpp::exceptions::InvalidParameterValueException Throws if parameter values are incorrect.
   */
  void parse_subscribe_topic(const std::string & subscribe_prefix);

  /**
   * @brief Parses data size string into a scalar and unit type
   *
   * Converts data size string (like "10M", "20K") into a scalar and unit type for easy calculations.
   *
   * @param [in] data_size The data size string (e.g., "10M", "20K").
   * @param [out] scalar Extracted scalar value.
   * @param [out] type Extracted size type (Bytes, KiloBytes, etc).
   * @return True if parsing successful, otherwise false.
   */
  bool parse_data_size(const std::string & data_size, float * scalar, SizeType * type);

  /**
   * @brief Publish callback.
   * @param [in] publisher The ROS2 publisher.
   * @param [in] msg_bytes The size of the message in bytes.
   * @param [in,out] seq_num The sequence number.
   */
  void pub_callback(
    rclcpp::Publisher<DummyMsgT>::SharedPtr publisher, uint64_t msg_bytes,
    int64_t & seq_num);

  /**
   * @brief Subscribe callback.
   * @param [in] msg Received message.
   * @param [in] topic_name The topic name.
   * @param [in,out] seq_num Sequence number.
   * @param [in,out] drop_msg_num Dropped message count.
   * @param [in,out] receive_num Received message count.
   * @param [in,out] initial_freq_time Initial frequency time for the topic.
   * @param [in,out] revieve_bytes Received bytes for the topic.
   */
  void sub_callback(
    const DummyMsgT::SharedPtr msg, const std::string & topic_name,
    int64_t & seq_num, int64_t & drop_msg_num, int64_t & receive_num,
    rclcpp::Time & initial_freq_time, size_t & revieve_bytes);

  /**
   * @brief Utility function for formatting bandwidth.
   * @param [in] byte Bandwidth in bytes.
   * @return Formatted bandwidth.
   */
  std::string bw_format(const size_t byte);

  bool m_root_node = false;  ///< Flag for checking if is root node.
  bool m_terminal_node = false;  ///< Flag for checking if is terminal node.
  std::vector<PublishTopic> m_publish_topics;  ///< Vector to hold all publish topics.
  std::vector<SubscribeTopic> m_subscribe_topics;  ///< Vector to hold all subscribe topics.
};

}  // namespace fabric_nodes

#endif  // FABRIC_NODES__DUMMY_NODE_HPP_

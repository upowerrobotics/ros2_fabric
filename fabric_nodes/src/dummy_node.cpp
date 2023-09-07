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

#include <fabric_nodes/dummy_node.hpp>

#include <algorithm>
#include <cstdio>
#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// The NOLINTs below are to fix Galactic bugs
#include <rclcpp/exceptions.hpp>  // NOLINT
#include <fabric_interfaces/msg/dummy_message.hpp>  // NOLINT

using namespace std::chrono_literals;

/**
 * @brief Calculates the length of a null-terminated C-style string.
 * @param [in] str A pointer to the null-terminated C-style string.
 * @return The length of the string (number of characters) excluding the null terminator.
 */
int constexpr char_len(const char * str)
{
  return *str ? 1 + char_len(str + 1) : 0;
}

static constexpr size_t MSG_OVERHEAD =
  sizeof(DummyMsgT::timestamp) +
  sizeof(DummyMsgT::seq_num);  ///< The massage size that is over head.
static constexpr uint8_t PUBLISH_PREFIX_SIZE =
  char_len("publish_topics.");  ///< Size of the text "publish_topics."
static constexpr uint8_t SUBSCRIBE_PREFIX_SIZE =
  char_len("subscribe_topics.");  ///< Size of the text "subscribe_topics."

/**
 * @brief Namespace containing classes and functions for the fabric nodes.
 */
namespace fabric_nodes
{

/**
 * @brief Constructs a DummyNode for ROS2
 *
 * The constructor for the DummyNode, which inherits from rclcpp::Node.
 * It sets up publishers and subscribers based on parameters.
 *
 * @param [in] options ROS2 Node options.
 * @exception rclcpp::exceptions::InvalidParametersException Throws an exception if required parameters are missing.
 */
DummyNode::DummyNode(rclcpp::NodeOptions options)
: rclcpp::Node(
    "dummy_node",
    options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true)
)
{
  this->get_parameter("root_node", m_root_node);
  this->get_parameter("terminal_node", m_terminal_node);

  const auto publish_params_msg = this->list_parameters({"publish_topics"}, 3);
  const auto subscribe_params_msg = this->list_parameters({"subscribe_topics"}, 3);

  if (!m_terminal_node) {
    // If this is not a terminal node, look for published topics
    if (publish_params_msg.names.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No topics provided to publish.");
      return;
    }

    // For each prefix, parse all of the parameters
    for (const auto & prefix : publish_params_msg.prefixes) {
      parse_publish_topic(prefix);
    }
  }

  if (!m_root_node) {
    // If this is not a root node, look for subscribed topics
    if (subscribe_params_msg.names.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No topics provided to subscribe.");
      return;
    }

    // For each prefix, parse all of the parameters
    for (const auto & prefix : subscribe_params_msg.prefixes) {
      parse_subscribe_topic(prefix);
    }
  }
}

/**
 * @brief Parses and sets up a publish topic
 *
 * This function parses parameters to setup a topic that this node will publish to.
 *
 * @param [in] param_prefix The prefix string for parameters related to this topic.
 * @exception rclcpp::exceptions::InvalidParameterValueException Throws if parameter values are incorrect.
 */
void DummyNode::parse_publish_topic(const std::string & param_prefix)
{
  PublishTopic pub;
  pub.topic_name = param_prefix.substr(PUBLISH_PREFIX_SIZE, std::string::npos);
  RCLCPP_INFO(this->get_logger(), "Publish topic: %s", pub.topic_name.c_str());
  const auto prefix_params_msg = this->list_parameters({param_prefix}, 3);

  for (const auto & param : prefix_params_msg.names) {
    const auto & param_name_trimmed = param.substr(param_prefix.length() + 1, std::string::npos);
    bool param_format_invalid = false;

    if (param_name_trimmed == "bandwidth") {
      std::string bw_value{};
      this->get_parameter(param, bw_value);

      if (!parse_data_size(bw_value, &(pub.bandwidth_scalar), &(pub.bandwidth_size_type))) {
        param_format_invalid = true;
        break;
      }

      RCLCPP_INFO(
        this->get_logger(),
        "\tbandwidth: %.1f bytes/sec",
        pub.bandwidth_scalar * static_cast<uint64_t>(pub.bandwidth_size_type)
      );
    } else if (param_name_trimmed == "frequency") {
      this->get_parameter(param, pub.frequency);

      RCLCPP_INFO(this->get_logger(), "\tfrequency: %.1f msgs/sec", pub.frequency);
    } else if (param_name_trimmed == "msg_size") {
      std::string size_value{};
      this->get_parameter(param, size_value);

      if (!parse_data_size(size_value, &(pub.msg_size_scalar), &(pub.msg_size_type))) {
        param_format_invalid = true;
        break;
      }

      RCLCPP_INFO(
        this->get_logger(),
        "\tmsg size: %.1f bytes",
        pub.msg_size_scalar * static_cast<uint64_t>(pub.msg_size_type)
      );
    } else {
      param_format_invalid = true;
    }

    if (pub.bandwidth_scalar == 0.0f && pub.frequency == 0.0f && pub.msg_size_scalar == 0.0f) {
      throw rclcpp::exceptions::InvalidParametersException{param_prefix +
              ": each topic must have two of bandwidth, frequency, or msg_size."};
    }

    if (param_format_invalid) {
      throw rclcpp::exceptions::InvalidParameterValueException{param +
              " does not have a valid format."};
    }
  }

  // TODO(jwhitleywork): Figure out QoS settings
  pub.publisher = this->create_publisher<DummyMsgT>(pub.topic_name, rclcpp::QoS{1});

  float frequency = pub.frequency;
  uint64_t msg_bytes =
    static_cast<uint64_t>(pub.msg_size_scalar * static_cast<float>(pub.msg_size_type));
  uint64_t bw_bytes =
    static_cast<uint64_t>(pub.bandwidth_scalar * static_cast<float>(pub.bandwidth_size_type));

  // If frequency is 0, must have only bw and msg size
  if (frequency == 0.0f) {
    frequency = static_cast<float>(bw_bytes) / static_cast<float>(msg_bytes);
  }

  // If msg size is 0, must have only frequency and bw
  if (msg_bytes == 0) {
    msg_bytes = static_cast<uint64_t>(static_cast<float>(bw_bytes) / frequency);
  }

  std::chrono::milliseconds timer_interval =
    std::chrono::milliseconds(static_cast<uint64_t>(1.0f / frequency * 1000.0f));

  // This is really stupid. See
  // https://answers.ros.org/question/308386/ros2-add-arguments-to-callback/
  // for explanation of the next line.
  std::function<void()> bound_func =
    std::bind(&DummyNode::pub_callback, this, pub.publisher, msg_bytes, pub.seq_num);

  pub.publish_timer = this->create_wall_timer(timer_interval, bound_func);

  m_publish_topics.push_back(std::move(pub));
}

/**
 * @brief Parses and sets up a subscribe topic
 *
 * This function parses parameters to setup a topic to which this node will subscribe.
 *
 * @param [in] subscribe_prefix The prefix string for parameters related to this topic.
 * @exception rclcpp::exceptions::InvalidParameterValueException Throws if parameter values are incorrect.
 */
void DummyNode::parse_subscribe_topic(const std::string & subscribe_prefix)
{
  SubscribeTopic sub;
  sub.topic_name = subscribe_prefix.substr(SUBSCRIBE_PREFIX_SIZE, std::string::npos);
  RCLCPP_INFO(this->get_logger(), "Subscribe topic: %s", sub.topic_name.c_str());
  const auto prefix_params_msg = this->list_parameters({subscribe_prefix}, 3);

  for (const auto & param : prefix_params_msg.names) {
    const auto & param_name_trimmed =
      param.substr(subscribe_prefix.length() + 1, std::string::npos);
    bool param_format_invalid = false;

    if (param_name_trimmed == "node") {
      this->get_parameter(param, sub.node_name);

      if (sub.node_name.length() < 1) {
        param_format_invalid = true;
        break;
      }

      RCLCPP_INFO(this->get_logger(), "\tnode name: %s", sub.node_name.c_str());
    } else {
      param_format_invalid = true;
    }

    if (param_format_invalid) {
      throw rclcpp::exceptions::InvalidParameterValueException{param +
              " does not have a valid format."};
    }
  }

  std::ostringstream topic_oss{};
  topic_oss << "/" << sub.topic_name;

  auto sub_options = rclcpp::SubscriptionOptions();
  sub_options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;

  sub.initial_freq_time = this->now();

  std::function<void(const DummyMsgT::SharedPtr)> cb = std::bind(
    &DummyNode::sub_callback, this, std::placeholders::_1,
    topic_oss.str(), sub.seq_num, sub.drop_msg_num, sub.receive_num,
    sub.initial_freq_time, sub.revieve_bytes);

  sub.subscriber = this->create_subscription<DummyMsgT>(
    topic_oss.str(), rclcpp::QoS{1}, cb, sub_options);

  m_subscribe_topics.push_back(std::move(sub));
}

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
bool DummyNode::parse_data_size(const std::string & data_size, float * scalar, SizeType * type)
{
  if (data_size.length() == 0) {return false;}

  const auto size_scalar = data_size.substr(0, data_size.length() - 1);
  const auto size_type = data_size.substr(data_size.length() - 1, std::string::npos);

  if (size_scalar.length() < 1 || size_type.length() != 1) {return false;}

  switch (size_type[0]) {
    case 'B':
      *type = SizeType::BYTES;
      break;
    case 'K':
      *type = SizeType::KILOBYTES;
      break;
    case 'M':
      *type = SizeType::MEGABYTES;
      break;
    case 'G':
      *type = SizeType::GIGABYTES;
      break;
    default:
      return false;
  }

  *scalar = std::stof(size_scalar);

  return true;
}

/**
 * @brief Publish callback.
 * @param [in] publisher The ROS2 publisher.
 * @param [in] msg_bytes The size of the message in bytes.
 * @param [in,out] seq_num The sequence number.
 */
void DummyNode::pub_callback(
  rclcpp::Publisher<DummyMsgT>::SharedPtr publisher, uint64_t msg_bytes,
  int64_t & seq_num)
{
  auto msg = std::make_unique<DummyMsgT>();
  msg->data.resize(msg_bytes - MSG_OVERHEAD, 42);
  msg->timestamp = this->now();
  msg->seq_num = seq_num;
  seq_num++;

  publisher->publish(std::move(msg));
}

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
void DummyNode::sub_callback(
  const DummyMsgT::SharedPtr msg, const std::string & topic_name,
  int64_t & seq_num, int64_t & drop_msg_num, int64_t & receive_num,
  rclcpp::Time & initial_freq_time, size_t & revieve_bytes)
{
  // Calculate ROS xmt time
  auto now = this->now();
  auto xmt_diff = now - rclcpp::Time(msg->timestamp);

  // Calculate Recieve Rate
  if (msg->seq_num != seq_num) {
    drop_msg_num += (msg->seq_num - seq_num);
    seq_num = msg->seq_num;
  } else {
    seq_num++;
  }
  float recieve_rate = (msg->seq_num != 0) ?
    (static_cast<float>(msg->seq_num - drop_msg_num) / msg->seq_num) : 1.0;

  // Calculate Frequency and Bandwidth
  revieve_bytes += (msg->data.size() + MSG_OVERHEAD);
  receive_num++;
  auto time_from_start = now - initial_freq_time;
  auto freq = receive_num / time_from_start.seconds();
  auto bandwidth = bw_format(revieve_bytes / time_from_start.seconds());

  RCLCPP_DEBUG(
    this->get_logger(),
    "Topic: %s, ROS xmt time ns: %li. ROSPUB TS: %li, ROSSUB TS: %li, "
    "Drop Num: %li, Recieve Rate: %f, Freq: %f, Bandwidth: %s",
    topic_name.c_str(), xmt_diff.nanoseconds(),
    rclcpp::Time(msg->timestamp).nanoseconds(), now.nanoseconds(),
    drop_msg_num, recieve_rate, freq, bandwidth.c_str());
}

/**
 * @brief Utility function for formatting bandwidth.
 * @param [in] byte Bandwidth in bytes.
 * @return Formatted bandwidth.
 */
std::string DummyNode::bw_format(const size_t byte)
{
  if (byte < static_cast<size_t>(SizeType::KILOBYTES)) {
    return std::to_string(byte) + "B";
  } else if (byte < static_cast<size_t>(SizeType::MEGABYTES)) {
    return std::to_string(byte / static_cast<size_t>(SizeType::KILOBYTES)) + "KB";
  } else if (byte < static_cast<size_t>(SizeType::GIGABYTES)) {
    return std::to_string(byte / static_cast<size_t>(SizeType::MEGABYTES)) + "MB";
  } else {
    return std::to_string(byte / static_cast<size_t>(SizeType::GIGABYTES)) + "GB";
  }
}

}  // namespace fabric_nodes

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(fabric_nodes::DummyNode)

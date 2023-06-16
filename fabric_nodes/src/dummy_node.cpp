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

#include <string>
#include <utility>
#include <vector>

#include <rclcpp/exceptions.hpp>  // NOLINT

// Size of the text "publish_topics."
static constexpr uint8_t PUBLISH_PREFIX_SIZE = 15U;
// Size of the text "subscribe_topics."
static constexpr uint8_t SUBSCRIBE_PREFIX_SIZE = 17U;

namespace fabric_nodes
{

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
      RCLCPP_ERROR(this->get_logger(), "No topics provided to subscribe to.");
      return;
    }

    // For each prefix, parse all of the parameters
    for (const auto & prefix : subscribe_params_msg.prefixes) {
      parse_subscribe_topic(prefix);
    }
  }
}

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
    } else if (param_name_trimmed == "msg_frequency") {
      this->get_parameter(param, pub.msg_frequency);

      RCLCPP_INFO(this->get_logger(), "\tfrequency: %.1f msgs/sec", pub.msg_frequency);
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

    if (pub.bandwidth_scalar == 0.0f && pub.msg_frequency == 0.0f && pub.msg_size_scalar == 0.0f) {
      throw rclcpp::exceptions::InvalidParametersException{param_prefix +
              ": each topic must have two of bandwidth, msg_frequency, or msg_size."};
    }

    if (param_format_invalid) {
      throw rclcpp::exceptions::InvalidParameterValueException{param +
              " does not have a valid format."};
    }
  }

  m_publish_topics.push_back(std::move(pub));
}

void DummyNode::parse_subscribe_topic(const std::string & param_prefix)
{
  SubscribeTopic sub;
  sub.topic_name = param_prefix.substr(SUBSCRIBE_PREFIX_SIZE, std::string::npos);
  RCLCPP_INFO(this->get_logger(), "Subscribe topic: %s", sub.topic_name.c_str());
  const auto prefix_params_msg = this->list_parameters({param_prefix}, 3);

  for (const auto & param : prefix_params_msg.names) {
    const auto & param_name_trimmed = param.substr(param_prefix.length() + 1, std::string::npos);
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

  m_subscribe_topics.push_back(std::move(sub));
}

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

}  // namespace fabric_nodes

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(fabric_nodes::DummyNode)

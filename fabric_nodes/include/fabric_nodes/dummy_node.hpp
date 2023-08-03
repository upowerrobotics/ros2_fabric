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

using DummyMsgT = fabric_interfaces::msg::DummyMessage;

namespace fabric_nodes
{

enum class SizeType
{
  BYTES = 1LL,
  KILOBYTES = 1024LL,
  MEGABYTES = 1048576LL,
  GIGABYTES = 1073741824LL
};

struct PublishTopic
{
  float bandwidth_scalar = 0.0f;
  SizeType bandwidth_size_type = SizeType::BYTES;
  float frequency = 0.0f;
  float msg_size_scalar = 0.0f;
  SizeType msg_size_type = SizeType::BYTES;
  std::string topic_name = "";
  rclcpp::Publisher<DummyMsgT>::SharedPtr publisher;
  rclcpp::TimerBase::SharedPtr publish_timer;
};

struct SubscribeTopic
{
  std::string node_name = "";
  std::string topic_name = "";
  rclcpp::Subscription<DummyMsgT>::SharedPtr subscriber;
};

class DummyNode : public rclcpp::Node
{
public:
  explicit DummyNode(rclcpp::NodeOptions options);

private:
  void parse_publish_topic(const std::string & param_prefix);
  void parse_subscribe_topic(const std::string & subscribe_prefix);
  bool parse_data_size(const std::string & data_size, float * scalar, SizeType * type);
  void pub_callback(rclcpp::Publisher<DummyMsgT>::SharedPtr publisher, uint64_t msg_bytes);
  void sub_callback(const DummyMsgT::SharedPtr msg, const std::string & topic_name);
  
  int64_t message_id_ = 0;
  int64_t pre_id = 0;
  int64_t catch_msg = 0;
  bool m_root_node = false;
  bool m_terminal_node = false;
  std::vector<PublishTopic> m_publish_topics;
  std::vector<SubscribeTopic> m_subscribe_topics;
};

}  // namespace fabric_nodes

#endif  // FABRIC_NODES__DUMMY_NODE_HPP_

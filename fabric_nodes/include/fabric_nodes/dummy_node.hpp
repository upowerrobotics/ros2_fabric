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

#include <rclcpp/rclcpp.hpp>  // NOLINT

namespace fabric_nodes
{

class DummyNode : public rclcpp::Node
{
public:
  explicit DummyNode(const rclcpp::NodeOptions options);
};

}  // namespace fabric_nodes

#endif  // FABRIC_NODES__DUMMY_NODE_HPP_

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

#include "fabric_gui/fabric_gui_node.h"
#include <QApplication>
#include <QObject>

FabricGUINode::FabricGUINode(rclcpp::NodeOptions options)
: rclcpp::Node("fabric_gui_node")
{
  int argc = 0;
  char ** argv = nullptr;
  app_ = std::make_unique<QApplication>(argc, argv);
  gui_ = std::make_unique<FabricGUI>();

  QObject::connect(
    gui_.get(), &FabricGUI::closeRequested, [this]() {
      this->handleCloseRequest();
    });

  gui_->show();
  app_->exec();
}

FabricGUINode::~FabricGUINode()
{
  gui_->close();
  rclcpp::shutdown();
}

void FabricGUINode::handleCloseRequest()
{
  rclcpp::shutdown();
}

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(FabricGUINode)

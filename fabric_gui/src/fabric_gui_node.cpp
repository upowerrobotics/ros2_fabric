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

FabricGUINode::FabricGUINode(rclcpp::NodeOptions options)
  : rclcpp::Node("fabric_gui_node") {
    int argc = 0;
    char **argv = nullptr;
    QApplication app(argc, argv);
    
    gui_ = std::make_unique<FabricGUI>();
    gui_->show();

    app.exec();
}

FabricGUINode::~FabricGUINode() {
    // Destructor
}

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(FabricGUINode)

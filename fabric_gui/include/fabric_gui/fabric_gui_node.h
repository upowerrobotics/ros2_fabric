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

#ifndef FABRIC_GUI_FABRIC_GUI_NODE_H
#define FABRIC_GUI_ABRIC_GUI_NODE_H

#include <rclcpp/rclcpp.hpp>
#include "fabric_gui/fabric_gui.h"

class FabricGUINode: public rclcpp::Node {
public:
  explicit FabricGUINode(rclcpp::NodeOptions options);
  ~FabricGUINode();

private:
  std::unique_ptr <QApplication> app_;
  std::unique_ptr <FabricGUI> gui_;

private slots:
  void handleCloseRequest();
};

#endif // FABRIC_GUI_NODE_H

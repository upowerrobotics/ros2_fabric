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

/**
 * @brief Represents a ROS 2 Node for the Fabric GUI.
 *
 * This class inherits from rclcpp::Node and is responsible for managing
 * the Fabric GUI application.
 */
class FabricGUINode: public rclcpp::Node {
public:
  /**
   * @brief Constructor for the FabricGUINode class.
   *
   * @param options NodeOptions for configuring the node.
   */
  explicit FabricGUINode(rclcpp::NodeOptions options);

  /**
   * @brief Destructor for the FabricGUINode class.
   */
  ~FabricGUINode();

private:
  /**
   * @brief Pointer to the Qt application.
   */
  std::unique_ptr < QApplication > app_;

  /**
   * @brief Pointer to the Fabric GUI instance.
   */
  std::unique_ptr < FabricGUI > gui_;

private slots:
  /**
   * @brief Slot function to handle a close request.
   *
   * This slot is called when the GUI window is closed.
   */
  void handleCloseRequest();
};

#endif // FABRIC_GUI_NODE_H

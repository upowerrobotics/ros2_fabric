# Copyright 2023 U Power Robotics, USA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(fabric_gui)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(POLICY CMP0071)
  cmake_policy(SET CMP0071 NEW)
endif()

# Explicitly find Qt5 first
find_package(Qt5 COMPONENTS Widgets REQUIRED)

# Find required dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

# Ensure Qt settings
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(FABRIC_GUI_SRC
  src/fabric_gui.cpp
  src/fabric_gui_node.cpp
  src/fabric_gui.ui
)  

set(FABRIC_GUI_HEADERS
  include/fabric_gui/fabric_gui.h
  include/fabric_gui/fabric_gui_node.h
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${FABRIC_GUI_SRC}
  ${FABRIC_GUI_HEADERS}
)

# Explicitly link with Qt libraries if necessary (might not be required, but kept for clarity)
target_link_libraries(${PROJECT_NAME}_node
  Qt5::Widgets
)

# Add the executable with ament_auto
rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "FabricGUINode"
  EXECUTABLE ${PROJECT_NAME}_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install necessary files (Modify according to your needs)
ament_auto_package(INSTALL_TO_SHARE
)

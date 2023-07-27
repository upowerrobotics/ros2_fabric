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

project(fabric_nodes)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(${PROJECT_NAME}_INCLUDES
  include/fabric_nodes/dummy_node.hpp
)

set(${PROJECT_NAME}_SRCS
  src/dummy_node.cpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_INCLUDES}
  ${${PROJECT_NAME}_SRCS}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "fabric_nodes::DummyNode"
  EXECUTABLE "dummy_node_exe"
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  # set(TEST_SOURCES test/test_ars430_driver.cpp)
  # set(TEST_ARS430_DRIVER_EXE test_ars430_driver)
  # ament_add_gtest(${TEST_ARS430_DRIVER_EXE} ${TEST_SOURCES})
  # target_link_libraries(${TEST_ARS430_DRIVER_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package(
  INSTALL_TO_SHARE
    config
    docs
    param
)

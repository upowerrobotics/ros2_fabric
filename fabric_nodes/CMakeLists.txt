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

find_package(Doxygen)
if(DOXYGEN_FOUND)
  # Set input and output files
  set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile)
  set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/docs/Doxyfile)

  # Request to configure the file
  configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

  # Note the option ALL which allows to build the docs together with the application
  add_custom_target(doc_doxygen ALL
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM)
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

ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  ${PROJECT_NAME}/get_log.py
  DESTINATION lib/${PROJECT_NAME}
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
  set(ament_cmake_flake8_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # launch test
  add_ros_test(
    tests/test_fabric_nodes_launch.py
    TIMEOUT "30"
  )

  # pytests
  set(_pytest_tests
    tests/test_config2node.py
    tests/test_node_list.py
    tests/test_topic_list.py
    # Add other test files here
  )
  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 60
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
  endforeach()

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
    test_config
    launch
    tests
)

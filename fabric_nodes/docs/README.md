@mainpage
# FABRIC User Documentation

This document describes how to operate the
[ROS2](https://www.ros.org) package named **fabric_nodes**.

FABRIC is a tool designed for the evaluation of end-to-end transmission statistics in a ROS environment.  
This package allows users to establish a virtual environment that mirrors their implementation,
facilitating the measurement of transmission latency, frequency, and bandwidth in the ROS2 layer.

FABRIC is compatible with any middleware boasting an RMW implementation with topic support.
Furthermore, it is capable of gauging latency within the rmw layer.
However, measuring latency in the RMW layer requires FABRIC-specific hooks
which is why we have our own forks of the RMW implementations.  

To measure latency in the rmw layer,
users are required to download and compile our own forks of the RMW implementations for each middleware.
Our own forks of the RMW implementations: 
[rmw_cyclonedds](https://github.com/upowerrobotics/rmw_cyclonedds),
[rmw_fastrtps](https://github.com/upowerrobotics/rmw_fastrtps),
and [rmw_ecal](https://github.com/upowerrobotics/rmw_ecal).

# Quick Start
- Launch `fabric_node` in the terminal.  
  After waiting for roughly 1 minute, kill the node:
```bash
ros2 launch fabric_nodes fabric_nodes.launch.py
```
  
- Run get_log.py to retrieve the evaluation.  
  The analyized csv file will be saved in the folder where the user runs this command:  
```bash
ros2 run fabric_nodes get_log.py
```
  
# System Structure Workflow (Default Config File)

The system follows a sequential workflow to process
and output various metrics related to latency, frequency, and bandwidth.
Here's a step-by-step breakdown:

## Initialization:

Execute [`fabric_nodes.launch.py`](../launch/fabric_nodes.launch.py) which:

- Reads the configuration from [`example.yaml`](../config/example.yaml).
  - For details on the config structure, refer to [YAML_API.md](YAML_API.md).
- Validates the yaml structure with [`config2node.py`](../fabric_nodes/config2node.py).
- Sets up nodes and topics using [`dummy_node.cpp`](../src/dummy_node.cpp).

## Output Metrics:

[`dummy_node.cpp`](../src/dummy_node.cpp) outputs metrics:

- Metrics: latency, frequency, and bandwidth.
- Logging: Uses [RCLCPP_DEBUG](https://docs.ros2.org/bouncy/api/rclcpp/logging_8hpp.html).
- Location: ROS log folder (by default ~/.ros/log on Linux systems)

## RMW Layer Output (Conditional):

- If the user installs the custom `rmw_implementation`,
  the `rmw_layer` also logs latency, frequency, and bandwidth.
  - Logging: Uses [RCLCPP_DEBUG](https://docs.ros2.org/bouncy/api/rclcpp/logging_8hpp.html).
  - Location: ROS log folder (by default ~/.ros/log on Linux systems)

## Log Extraction:

- [`get_log.py`](../fabric_nodes/get_log.py) processes the logs:

  - Reads from ROS log folder (by default ~/.ros/log on Linux systems).
  - Extracts: latency, frequency, and bandwidth for each topic.
  - Save the analyized csv file to where the user runs `get_log.py`.

# User Configuration YAML for Evaluation

Follow these steps to set up and utilize your custom `config.yaml`:

## Setup Configuration
- Create a `config.yaml` tailored to your environment.
- Ensure it adheres to the required format;
  otherwise, [`config2node.py`](../fabric_nodes/config2node.py) will throw a `ValueError`.
- Each environment object defines a compute environment, whether that is an Operating System, 
  SoC, ECU, or ROS environment. All nodes in a single environment will be launched together.
- For more details on the `config.yaml` structure, please refer to [YAML_API.md](YAML_API.md)

## Execute Configuration
- Run the command below to apply your `config.yaml`.
- Use the environment argument to set the compute environment.
- Make sure killing the node after 1 minute.
```bash
ros2 launch fabric_nodes fabric_nodes.launch.py config-path:=/PATH/TO/USER/CONFIG environment:=USER_ENV
```

## Extract Measurements
To fetch the measurements from the ROS log folder (by default ~/.ros/log on Linux systems), use:
```bash
ros2 run fabric_nodes get_log.py
```

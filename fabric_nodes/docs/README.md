# FABRIC User Documentation

This document describes how to operate the
[ROS2](https://www.ros.org) package named [fabric_nodes](https://github.com/upowerrobotics/ros2_fabric).

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

# 0. Quick Start (with UI)
- Run **fabric_gui** in the terminal.
<div style="width: auto; white-space: nowrap;">
<pre>
ros2 run fabric_gui fabric_gui_exe
</pre>
</div>

- Button Discription:
  - Select Workspace: Select the current ROS2 workspace you are using.
  - Select Configuration File: Select the yaml configuration file you are using. Refer to \ref YAML_API.
  - Fabric Launch: Runs the **fabric_nodes** for 60 sec and plot the data.
  - Pause: pause the **fabric_nodes** and plot the data.

- Tab Widget Discription:
  - Measurement Result: Shows the rawdata of FABRIC.
  - Average Result: Shows the average of rawdata of FABRIC.

# 1. Quick Start (with command)
- Launch **fabric_node** in the terminal.  
  After waiting for roughly 1 minute, kill the node:
<div style="width: auto; white-space: nowrap;">
<pre>
ros2 launch fabric_nodes fabric_nodes.launch.py
</pre>
</div>
  
- Run get_log.py to retrieve the evaluation.  
  The analyized csv file will be saved in the folder where the user runs this command:  
<div style="width: auto; white-space: nowrap;">
<pre>
ros2 run fabric_nodes get_log.py
</pre>
</div>
  
# 2. System Structure Workflow (Default Config File)

The system follows a sequential workflow to process
and output various metrics related to latency, frequency, and bandwidth.
Here's a step-by-step breakdown:

## 2-1. Initialization:

Execute **fabric_nodes.launch.py** which:

- Reads the configuration from **config/example.yaml**:
  - For details on the config structure, refer to \ref YAML_API.
- Validates the yaml structure with \ref fabric_nodes.config2node.Config2Nodes "Config2Nodes Class".
- Sets up nodes and topics using \ref fabric_nodes::DummyNode "DummyNode Class".

## 2-2. Output Metrics:

\ref fabric_nodes::DummyNode "DummyNode Class" outputs metrics:

- Metrics: latency, frequency, and bandwidth.
- Logging: Uses [RCLCPP_DEBUG](https://docs.ros2.org/bouncy/api/rclcpp/logging_8hpp.html).
- Location: ROS log folder (by default ~/.ros/log on Linux systems)

## 2-3. RMW Layer Output (Conditional):

- If the user installs the custom `rmw_implementation`,
  the **rmw_layer** also logs latency, frequency, and bandwidth.
  - Logging: Uses [RCLCPP_DEBUG](https://docs.ros2.org/bouncy/api/rclcpp/logging_8hpp.html).
  - Location: ROS log folder (by default ~/.ros/log on Linux systems)

## 2-4. Log Extraction:

- \ref fabric_nodes.get_log.GetLog "GetLog Class" processes the logs:

  - Reads from ROS log folder (by default ~/.ros/log on Linux systems).
  - Extracts: latency, frequency, and bandwidth for each topic.
  - Save the analyized csv file to where the user runs \ref fabric_nodes.get_log.GetLog "GetLog Class".

# 3. User Configuration YAML for Evaluation

Follow these steps to set up and utilize your custom **config.yaml**:

## 3-1. Setup Configuration
- Create a **config.yaml** tailored to your environment.
- Ensure it adheres to the required format;
  otherwise, \ref fabric_nodes.config2node.Config2Nodes "Config2Nodes Class" will throw a **ValueError**.
- Each environment object defines a compute environment, whether that is an Operating System, 
  SoC, ECU, or ROS environment. All nodes in a single environment will be launched together.
- For more details on the **config.yaml** structure, please refer to \ref YAML_API.

## 3-2. Execute Configuration
- Run the command below to apply your **config.yaml**.
- Use the environment argument to set the compute environment.
- Make sure killing the node after 1 minute.
<div style="width: auto; white-space: nowrap;">
<pre>
ros2 launch fabric_nodes fabric_nodes.launch.py config-path:=/PATH/TO/USER/CONFIG environment:=USER_ENV
</pre>
</div>

## 3-3. Extract Measurements
To fetch the measurements from the ROS log folder (by default ~/.ros/log on Linux systems), use:
<div style="width: auto; white-space: nowrap;">
<pre>
ros2 run fabric_nodes get_log.py
</pre>
</div>

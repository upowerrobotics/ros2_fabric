\page YAML_API ROS2 FABRIC YAML API

This document describes the format of YAML documents which can be read and acted upon by the FABRIC
Python Startup script. See the **config** folder for example
configurations which conform to this specification.

## How to Read this Document

The YAML file is divided into objects and properties. Objects have properties and each property
of an object is written in the following form:

- **property_name** *[property_value_type]* **required**: Property description.

## Document Structure

### Root Node

The root node of a FABRIC YAML document must always be **environments:**. It must contain an array
of one or more environment objects.

### Environment Objects

Each environment object defines a compute environment, whether that is an Operating System, SoC,
ECU, or ROS environment. All nodes in a single environment will be launched together.

Properties:

- **name** *[string]* **required**: A name for the environment containing only alpha-numeric
  characters which must be unique in the document.

- **nodes** *[array: Node Objects]* **required**: An array of one or more node objects.

### Node Objects

Each node object represents a single process in an environment.

Properties:

- **name** *[string]* **required**: The name of this node or the base of the name of each node in
  this group of nodes if **qty** is greater than 1. Must contain only alpha-numeric characters and be
  unique in the environment.

- **qty** *[integer]*: The number of nodes to create with identical properties. If this property is
  not included the value is assumed to be 1.

- **root_node** *[boolean]* **required**: Accepts values of **true**, **True**, **false**, or **False**.
  Indicates whether or not this node is at the beginning of the pub/sub architecture. Root nodes
  can only contain publishers and not subscribers. A node can not be both a **root_node** and a
  **terminal_node**.

- **terminal_node** *[boolean]* **required**: Accepts values of **true**, **True**, **false**, or **False**.
  Indicates whether or not this node is at the end of the pub/sub architecture. Terminal nodes can
  only contain subscribers and not publishers. A node can not be both a `root_node` and a
  **terminal_node**.

- **publishers** *[array: Publisher objects]*: An array of one or more publisher objects. Required if
  `terminal_node` is false.

- `subscribers` *[array: Subscriber objects]*: An array of one or more subscriber objects. Required
  if **root_node** is false.

### Publisher Objects

Each publisher object represents one or more publishers on a node. Each publisher object must
contain exactly two of the three properties **bandwidth**, **frequency**, and **msg_size** and the third
will be inferred from the other two.

Properties:

- **name** *[string]* **required**: The name of this publisher or the base of the name of each
  publisher in this group of publishers if **qty** is greater than 1. Must contain only alpha-numeric
  characters and be unique in this node. Does not need to be unique in a group of nodes because
  topic names will be namespaced with the node name.

- **qty** *[integer]*: The number of publishers to create with identical properties. If this propery
  is not included the value is assumed to be 1.

- **bandwidth** *[string]*: The total bandwidth for this topic or a single instance of this topic if
  **qty** is greater than 1. This value is a combination of a float value and a single character
  indicating the unit. The character unit can be **B**, **K**, **M**, or **G** indicating Bytes, Kilobytes,
  Megabytes, or Gigabytes per second respectively. e.g. **2.3M** would be 2.3 Megabytes/second or
  2,411,725 Bytes/second (rounded to the nearest integer value).

- **frequency** *[float]*: The frequency of message publishing in messages/second.

- **msg_size** *[string]*: The size of the payload of each message transmission, not including a
  timestamp field. This value is a combination of a float value and a single character indicating
  the unit. The character unit can be **B**, **K**, **M**, or **G** indicating Bytes, Kilobytes, Megabytes,
  or Gigabytes respectively. e.g. **2.3M** would be 2.3 Megabytes or 2,411,725 Bytes (rounded to the
  nearest integer value).

- **QoS_depth** *[integer]*: This is the history depth and is relevant 
  when the **history_policy** is set to **RMW_QOS_POLICY_HISTORY_KEEP_LAST**. 
  The depth specifies the number of most recent messages that should be kept around. 
  For instance, if depth is set to 10, only the 10 most recent messages will be stored. 
  Any new incoming messages will overwrite the oldest stored messages. 
  If the depth is set to 0 the **history_policy** will be set to **RMW_QOS_POLICY_HISTORY_KEEP_ALL**. 

- **QoS_policy** *[string]*: The reliability policy of the QoS setting.
  The policy can either be **reliable** or **best_effort**.

### Subscriber Objects

Each subscriber object represents one subscriber on a node.

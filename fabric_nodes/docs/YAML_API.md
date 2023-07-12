# ROS2 FABRIC YAML API

This document describes the format of YAML documents which can be read and acted upon by the FABRIC
Python Startup script. See the [`config_examples`](../config_examples) folder for example
configurations which conform to this specification.

## How to Read this Document

The YAML file is divided into objects and properties. Objects have properties and each property
of an object is written in the following form:

- `property_name` [property_value_type] '(required)': Property description.

## Document Structure

### Root Node

The root node of a FABRIC YAML document must always be `environments:`. It must contain an array
of one or more environment objects.

### Environment Objects

Each environment object defines a compute environment, whether that is an Operating System, SoC,
ECU, or ROS environment. All nodes in a single environment will be launched together.

Properties:

- `name` [string] '(required)': A name for the environment containing only alpha-numeric characters
  which must be unique in the document.

- `nodes` [array: Node Objects] '(required)': An array of one or more node objects.

### Node Objects

Each node object represents a single process in an environment.

Properties:

- `name` [string] '(required)': The name of this node or the base of the name of each node in this
  group of nodes if `qty` is greater than 1. Must contain only alpha-numeric characters and be
  unique in the environment.

- `qty` [integer]: The number of nodes to create with identical properties. If this property is not
  included the value is assumed to be 1.

- `root_node` [boolean] '(required)': Accepts values of `true`, `True`, `false`, or `False`.
  Indicates whether or not this node is at the beginning of the pub/sub architecture. Root nodes
  can only contain publishers and not subscribers. A node can not be both a `root_node` and a
  `terminal_node`.

- `terminal_node` [boolean] '(required)': Accepts values of `true`, `True`, `false`, or `False`.
  Indicates whether or not this node is at the end of the pub/sub architecture. Terminal nodes can
  only contain subscribers and not publishers. A node can not be both a `root_node` and a
  `terminal_node`.

- `publishers` [array: Publisher objects]: An array of one or more publisher objects. Required if
  `terminal_node` is false.

- `subscribers` [array: Subscriber objects]: An array of one or more subscriber objects. Required
  if `root_node` is false.

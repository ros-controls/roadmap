extract ros2 pub/sub/services into a `ros2_communication` helper files. 

## Motivation:
In a project, we're wrapping `controller_manager` in a custom scheduler task, essentially re-writing the `ros2_control_node`

- **Conceptual ambiguity**: `ros2_control_node` is not a node, but an executable specific to the OS scheduler responsible for instantiating and running the `controller_manager`, which IS the node.
- **Code Maintainability**: `controller_manager.cpp` file is hitting 4k LoC, mixes core logic and ROS2 communicaiton (services, publishers, subscribers)

## Proposed changes
We propose a simple refactoring with two main objectives:

1. **Extract all ROS 2 interfaces** and their callback implementations into a helper file `controller_manager_ros.cpp/hpp` which current `controller_manager.hpp` would include.

2. **Rename the current `ros2_control_node`** to `ros2_control_executable` to accurately reflect its function as a system entry point rather than a Node

## Notes:
naming is up for discussion, of course

#### Food for thought:
Outside of scope for this suggestion, but we could push it a step further and make `ControllerManager` not inherit from `Node` at all, and place all ROS-related stuff into `ControllerManagerNode` class.

Then, as we do with `ResourceManager`, we'd pass in the necessary node interfaces to it:
- executor (already passed)
- parameters
- logger
- clock

Then the `ControlNode` would be responsible for ros2 communication, controller manager for controllers and resource manager for hardware.
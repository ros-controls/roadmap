extract ros2 pub/sub/services into a `ros2_communication` helper files. 

## Motivation:
In a project, we're wrapping `controller_manager` in a custom scheduler task, essentially re-writing the `ros2_control_node`

- **Conceptual ambiguity**: `ros2_control_node` is not a node, but an executable specific to the OS scheduler responsible for instantiating and running the `controller_manager`, which IS the node.
- **Code Maintainability**: `controller_manager.cpp` file is hitting 4k LoC, mixes core logic and ROS2 communicaiton (services, publishers, subscribers)

## Proposed changes
We propose a simple refactoring with two main objectives:

1. **Extract all ROS 2 interfaces** and their callback implementations into a helper file `ros2_communication.cpp/hpp` which current `controller_manager.hpp` would include.

2. **Rename the current `ros2_control_node`** to `ros2_control_executable` to accurately reflect its function as a system entry point rather than a Node
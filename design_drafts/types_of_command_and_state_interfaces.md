# Extension of types used for command and state interfaces

For introduction the file about general [purpose inputs/outputs in ros2_control](non_joint_command_interfaces.md).

## Motivation
Many GPIO values used in modern robots are logical and not numeric values.
For example, a vacuum valve at the robot has "on" (true) and "off" (false) states.

## Problem
Currently, the `ros2_control` framework supports only the `double` type to exchange data between hardware and controllers.
This is confusing for users and it would be clearer if also boolean and integer values can be transported.

## Possible solution
Extend [`ReadOnlyHandle`](https://github.com/ros-controls/ros2_control/blob/93b15787f1d2e16dd41d202cebff5fdbef56e19d/hardware_interface/include/hardware_interface/handle.hpp#L31) to accept other primitive types like `bool`, `int`, and unsigned types.
This could be simply done by templating the class, or to keep it restricted implementing constructors, getters and setters for those types.
The latter could become complex to keep track about specific types.

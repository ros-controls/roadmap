# "One-shot"/"Trigger" command interfaces

# Motivation
In the ros_control and first version of ros2_control (in 2021) all commands to the hardware interface are periodic and continually sent to the driver.
This keeps the bandwidth to the hardware high because all commands are sent all the time.
There are use cases where a one-shot command should be sent to the robot driver, e.g., to turn on some digital output.
This would reduce at least general bandwidth to the hardware by reducing the communication to only necessary at each update.
Besides the presented example, there are other cases where it is useful to know in the hardware if and when commanded interfaces changed its value.
This is possible to achieve without an extension of framework's interface management, but additional logic and storage in hardware interface is needed.
Here is shown an approach that would realize described functionality without any additional complexities inside a hardware interface.

This is also a very useful feature for GPIO and async (taking more than one cycle to get set) commands because it provides feedback to the controller that some values are written to a hardware.

# Background Knowledge
ros2_control framework's original purpose is to transfer continuous commands, like position and velocity values, between a controller and a hardware interface.
This is done by sharing memory between controllers' outputs and hardware interfaces' inputs, enabling direct "writing" of commands.
The main drawback of this simple architecture is that hardware does not know if and when a command was updated; and a controller does not know if and when a hardware interfaces wrote the command to physical hardware.
For continuous interfaces where it is expected that hardware has to "reach" the commanded value, this is not an issue.


# Purpose and Use
The proposed functionality has two purposes:
  - adding additional information in the hardware if and when a command has changed; and
  - signaling to a controller that hardware interface has read and used a variable (async commands).
 
A one-shot, i.e., trigger command interfaces have additional argument `one_shot="true"` or `async="true"` to mark them.
Then, ros2_control URDF tag would then for example look like:

```
<ros2_control name="OneShotCommandInterfacesAvailabilityExample" type="system">
  <hardware>
    <plugin>cool_robot/VeryCoolRobot</plugin>
  </hardware>
  <joint name="joint1">
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <command_interface name="recover_from_fault" one_shot="true"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <param name="initial_position">3.45</param>
  </joint>
  <joint name="joint2">
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <command_interface name="recover_from_fault" one_shot="true"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <param name="initial_position">2.78</param>
  </joint>
  <gpio name="flange_analog_IOs">
    <command_interface name="analog_output1" data_type="double" sync="true"/>
    <state_interface name="analog_output1"/>
    <state_interface name="analog_input1"/>
    <state_interface name="analog_input2"/>
  </gpio>
  <gpio name="flange_vacuum">
    <command_interface name="vacuum" one_shot="true"/>
    <state_interface name="vacuum" data_type="double"/>
  </gpio>
</ros2_control>
```

In the above example, "digital" interface that can be set to hardware without a delay are marked with `one_shot` and digital interfaces that are written asynchronously from the update are marked with `async` parameter.
The interfaces for continuous control, like *position* and *velocity*, are not changed.
 
 
# Implementation
1. extension of `hardware_interface/component_parser.cpp` to understand `one_shot` and `async` arguments on (command) interfaces
1. extension of `hardware_interface/handle.hpp` to add this attribute into command interface handle (read/write handle)
1. extension of `hardware_interface/handle.hpp` with getters on the interface to automatically reset value to "NaN" when interface is read
1. propose extension of hardware interfaces to use "handle" and do not access directly to the values as it is currently done.


































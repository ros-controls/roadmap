# Movement/Safety-critical interfaces

# Background knowledge
This functionality is based on two core concepts of ros2_control framework, "Hardware Interfaces" and "Hardware Lifecycle".

A hardware interface is a "communication port" between a controller and a hardware. 
There are two types of interfaces, "command" interfaces that are used for controlling, i.e., writing data to, a robot; and "state" interfaces are providing information about, i.e., reading data from, a robot.
Both of those interface types have in most cases primitive values, e.g. double.

The hardware lifecycle implements a finite state-machine for each hardware interface, representing the different stages of activity a hardware can be in.
In general, there are two states of hardware lifecycle when its interfaces are available and can be claimed by controllers, INACTIVE and ACTIVE.
The concept presented here explains the difference between those two states and their influence on interface availability.

## Motivation

The motivation behind here-presented concepts comes from safety engineering and industrial robotics.
First, let's see how an industrial robot is controlled using teach pendent and what functionalities are available in each stage.
When you start an industrial robot it goes through *initialization* process, establishes connections with the hardware and *configures* itself, based on the actual hardware state and default or pre-stored configurations.
After that, hardware's (robot's) internal states are available for reading and additional configuration.
It is essential for this stat that hardware cannot be moved.
Regarding the lifecycle of hardware interfaces in ros2_contol framework, this state is called *INACTIVE*. 

After a robot is configured, its movements can be activated by pressing an enable button, which enables energy flow toward the actuators.
This stage is modeled in ros2_control using *on_acitvte* method of the hardware interface.


## Purpose and modeling of movement critical interfaces

ros2_contol framework enables availability of command interfaces in two stages, i.e., in *on_configure* and *on_activate* methods.
The purpose of this feature is to provide separation of "configuration" commanding interfaces and safety-relevant commanding interfaces, e.g., interfaces that influence the robot's movement.
In general, all interfaces are available in *INACTIVE* state, i.e., after *on_configure* method finishes successfully and before *on_cleanup* method is executed. 
The interfaces that should be available only in *ACTIVE* STATE, i.e., after *on_activate* is successful and before *on_deactivate* is executed, have to be marked with additional argument `only_available_when_active="true"`.

An "ros2_control" tag of URDF description of a robot utilizing this functionality would look like:

```
<ros2_control name="TwoStageCommandInterfacesAvailabilityExample" type="system">
  <hardware>
    <plugin>cool_robot/VeryCoolRobot</plugin>
  </hardware>
  <joint name="joint1">
    <command_interface name="position" only_available_when_active="true"/>
    <command_interface name="velocity" only_available_when_active="true"/>
    <command_interface name="recover_from_fault"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <param name="initial_position">3.45</param>
  </joint>
  <joint name="joint2">
    <command_interface name="position" only_available_when_active="true"/>
    <command_interface name="velocity" only_available_when_active="true"/>
    <command_interface name="recover_from_fault"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <param name="initial_position">2.78</param>
  </joint>
  <gpio name="flange_analog_IOs">
    <command_interface name="analog_output1" data_type="double"/>
    <state_interface name="analog_output1"/>
    <state_interface name="analog_input1"/>
    <state_interface name="analog_input2"/>
  </gpio>
  <gpio name="flange_vacuum">
    <command_interface name="vacuum" only_available_when_active="true"/>
    <state_interface name="vacuum" data_type="double"/>
  </gpio>
</ros2_control>
```

In this example, *position*, *velocity* and *vacuum* command interface are only available when hardware is activated.
All state interfaces and other command interfaces, like command *recover_from_fault* and *analog_input1* are available for controllers in *INACTIVE* and *ACTIVE* state.
This example also depicts the main purpose of two-stage interface availability and that is having active communication to the robot, but limiting its movement and all functionalities that are potentially dangerous for its surrounding on the *ACTIVE* state.


## Implementation

Some ideas about the implementation steps/needs for this feature:

1. extension of `hardware_interface/component_parser.cpp` to understand `only_available_when_active` on (command) interfaces
1. extension of `hardware_interface/handle.hpp` to add this attribute into command interface handle (read/write handle)
1. extend `hardware_interface/resource_manager.cpp::configure_hardware` to add only command interfaces that have `only_available_when_active == false`  to `available_command_interfaces_` list
1. extend `hardware_interface/resource_manager.cpp::activate_hardware` to add command interfaces that have `only_available_when_active == true`  to `available_command_interfaces_` list
1. extend `hardware_interface/resource_manager.cpp::deactivate_hardware` and `hardware_interface/resource_manager.cpp::cleanup_hardware` opposite of the above two points

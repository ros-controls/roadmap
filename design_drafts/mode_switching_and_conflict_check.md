# Mode switching and conflict checking

# Problem

## Summary

Joints can have multiple, mutually exclusive command interfaces.
We need to ensure that
* only one of them can be claimed at a time,
* the System that provides them is aware which one is active to facilitate actuator mode switching.
At the same time we should also allow multiple, non-exclusive command interfaces to be implemented.

With the current implementation of `ResourceManager`, all command interfaces are made available by each System when they `export_command_interfaces()`. This setup does not allow for returning/swapping out command interfaces on-the-fly nor any communication to System components other than through command interfaces. Furthermore, the `ResourceManager` has no knowledge which System component a given command interface was exported from.

## Example system definitions

These system definitions were taken from `components_archutecture_and_urdf_examples.md` for reference.

### 2. Industrial Robots with multiple interfaces (can not be written at the same time)
  * the communication is done using proprietary API to communicate with robot control box
  * Data for all joints is exchanged in batch (at once)
  * Examples: KUKA FRI, ABB Yumi, Schunk LWA4p, etc.

```xml
  <ros2_control name="RRBotSystemMultiInterface" type="system">
    <hardware>
      <plugin>ros2_control_demo_hardware/RRBotSystemMultiInterfaceHardware</plugin>
      <param name="example_param_write_for_sec">2</param>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <joint name="joint1">
      <command_interface name="position">
        <param name="min">-1</param>
        <param name="max">1</param>
      </command_interface>
      <command_interface name="velocity">
        <param name="min">-1</param>
        <param name="max">1</param>
      </command_interface>
      <command_interface name="effort">
        <param name="min">-0.5</param>
        <param name="max">0.5"</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="joint2">
      <command_interface name="position">
        <param name="min">-1</param>
        <param name="max">1</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
  </ros2_control>
```

### 2.1 Robots with multiple interfaces used at the same time - the same structure as in (2)
  * the communication is done using proprietary API to communicate with robot control box
  * Data for all joints is exchanged in batch (at once)
  * Multiple values can be commanded, e.g. goal position and the maximal velocity on the trajectory allowed
  * Examples: humanoid robots (TALOS), 4 legged robot [solo](https://github.com/open-dynamic-robot-initiative/master-board/blob/master/documentation/BLMC_%C2%B5Driver_SPI_interface.md)


## Background for joint control mode

### Classical control mode

A control mode for a joint is the low level control mode applied to an actuator. It can be realized by a power electronics board with a micro controller, an embedded computer with a real-time operating system, a composition of system.

The simplest system is usually the power electronics with a micro controller.
For a DC motor it typically implements a Pulse Width Modulation system taking as an input a voltage and providing current as an output.

The simplest control mode in this case is the so called effort mode in ROS-1 which is voltage multiply by a scalar to get current.

If an encoder is present in the motor side, one can compute the difference between the motor position and the desired one, implementing a position control mode.

With a sufficient encoder precision and with a high-frequency sampling (1KHz-10KHz) it is possible to compute the shaft velocity and implement a velocity control mode.

To summarize it is possible to have the following three modes in one actuator:
 * position
 * velocity
 * effort

However other control schemes exist. They are explained later on.

### Control mode in simulation

An important note is that position and velocity means a desired value send to a low level control system.

The time response, frequency range and stability margin depends heavily of the underlying power electronics and the gains of the control loop.
In simulation those two schemes can be simulated by a PID sending a joint torque which is closer to what one will obtain on the real robot.

Simulation can be used to check various level of reality.
At first, when one wants to check geometrical consistency and not the underlying subsystem of the actuator, the desired position can be set as a constraint in the simulation algorithm.
Then the PID simulation is irrelevant. Right now there is no way to make the difference between such two control modes.

The same is true for robot using a joint torque measurement to perform torque control.


### Control mode using a closed source controller

Industrial robot provides their own controller based on their integration of the power electronics and the best low level control scheme they found.

### Control mode using an open source controller
When the micro controller can be programmed, it is possible to implement a control mode where desired position, desired velocity, desired effort and low level control gains are specified. The control gains can be computed using a LQR approach for instance.
This is typically the case when using AC motor with power electronics (recent approach).

### General properties
A control mode is a control scheme applied to an actuator. It can use one or several state and control interfaces.
It is *NOT* possible to mix control mode together for one actuator.

To switch from one control mode to another control mode on one actuator, it is necessary to check if this is feasible from the viewpoint of the actuator itself.

Often it is dangerous to switch control mode for several actuators at once (due to self-collision for instance).
The robot has to be in a specific state. This is usually done at the system level (controllers + hardware) that such safety is enforced.
This can not be done at the control mode switching.

# Solutions

## Proposal 1

Supplied by @mahaarbo in https://github.com/ros-controls/ros2_control/pull/322

This approach broadcasts all resource claims and allows System components to act on it as they see fit. The API consists of the following two main calls:

```
return_type
ResourceManager::notify_command_resource_claim(const std::vector<std::string> & interfaces);
```
and
```
return_type
System::accept_command_resource_claim(const std::vector<std::string> & interfaces);
```
where the set of control interfaces needed for the mode is provided in the interface.

## Proposal 2

The mode switching logic introduced to `ros_control` in ROS Indigo using [`canSwitch()` and `doSwitch()`](https://github.com/ros-controls/ros_control/pull/200).

```
virtual bool canSwitch(const std::list<ControllerInfo> &start_list,
                       const std::list<ControllerInfo> &stop_list) const;

virtual void doSwitch(const std::list<ControllerInfo> &start_list,
                      const std::list<ControllerInfo> &stop_list);
```

TBC
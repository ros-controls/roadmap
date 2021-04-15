# Transmission loading and parametrization

## Motivation

The role of transmissions in `ros_control` and `ros2_control` is to implement value transformations that reflect the effect of different mechanical transmissions.
These implement the same underlying mathematical formulas that are used when modeling mechanical transmissions.

The value transformation can be responsible for something as simple as a reducer transmission, where a certain *joint-level* value is divided by a factor accounting for a mechanical reduction rate to compute the *actuator-level* command to be sent to the actuator while the inverse is applied for feedback from the actuator.


![ros2_control Simple Transmission][simple_transmission]


More complex mechanical transmissions involve more than one joint where per-joint reduction rate depends on the state of other joints in the system, such as a four-bar linkage  transmission.

![ros2_control FourBarLinkage Transmission][four_bar_linkage_transmission]

Disclaimer: The `transmission_interface` library is not always required. It mainly depends on the level of abstraction the hardware implements and makes available through the hardware-level communication interfaces. An industrial arm for instance will typically expose *joint-level* interfaces through their comms and will have individually actuated joints. A humanoid robot on the other hand may employ differential transmissions for implementing multi-axis joints often seen in wrists, four-bar-linkages for larger parts of the body, such as torso movement, or a complex leg with a single actuator but multiple joints with feedback from them, etc.


## How it works in ROS Noetic and problems

To facilitate extendability, transmissions are implemented as plugins that are loaded and configured at startup time. This allows anyone to implement their custom transmission plugins in any ROS package without having to change `ros_control`.

### XML format

An additional XML description for each joint has to be added to the URDF in order to define transmissions on them.

```
<?xml version="1.0"?>

<robot name="robot" xmlns="http://www.ros.org">

  <transmission name="four_bar_linkage_trans">
    <type>transmission_interface/FourBarLinkageTransmission</type>
    <joint name="bar_joint">
      <role>joint2</role>
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <joint name="foo_joint">
      <role>joint1</role>
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="foo_actuator">
      <role>actuator1</role>
      <mechanicalReduction>50</mechanicalReduction>
    </actuator>
    <actuator name="bar_actuator">
      <role>actuator2</role>
      <mechanicalReduction>-50</mechanicalReduction>
    </actuator>
  </transmission>

</robot>
```

Let's dissect the XML above:

* `<type>transmission_interface/FourBarLinkageTransmission</type>` defines the transmission plugin to be loaded, parameters are following
* `<joint name="bar_joint">` defines the role and hardware interface of joints. The joint `bar_joint` should already be defined in the URDF.
* `<role>joint2</role>` is internal to transmission implementations, we'll not explain that here
* `<hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>` defines the *joint interface type* which the transmission needs to know for picking the right formula to apply to the values, e.g. a reduction rate may have a different effect to position, velocity or effort values
* the section enclosed by `<actuator name="foo_actuator">` defines the actuator name, role and mechanical reduction rate to be used in the underlying formula.

Let's look at some issues now.

### More than one role

This is the first and only place *joint interface type* is defined for a joint. It should not be the responsibility of transmissions to carry this information but since the URDF has no support for it, it had to be added here.

Throughout ROS1 there are many robots using `SimpleTransmission` with a reduction rate of 1 only to be able to define the type of joint interfaces they use so `gazebo_ros_control` can use the intended joint interfaces and simulate their robot properly.

### Actuator names are not really defined

Actuators are virtually non-existent in the URDF, the first and only place they are shown is in the transmission definition. It is entirely up to the one implementing the `RobotHW` to make sense of these names.

Common policies are to match `bar_joint` to `bar_actuator` or `joint1` to `joint1_motor`.

### Joint and actuator tags

There are some minor issues that cause confusion from time to time:
* Why is the `hardwareInterface` tag paired with the joint? Could be defined for both if we want to allow different interfaces or if we want to ensure that they are not different for compatibility...
* Why is the `mechanicalReduction` tag paired with the actuator? The reduction is applied to one of them when the information flows from actuator to controller but the inverse is applied when it flows from controller to actuator.



## Proposals for ROS2 XML format

The plugin-based system should be kept for transmissions. The tag `type` was replaced with `plugin` in the xml format for clarity and consistency with the rest of `ros2_control`.

The parametrization should become simpler as it doesn't need to concern itself anymore with the joint command interface types as they are defined elsewhere.

It is however still unclear if there is any use to define actuator names outside the transmission scope.

Where to place `mechanical_reduction`?

### Proposal 1

Merging the current ros2_control xml format with the ROS1 transmissions:

```xml
  <ros2_control name="RRBotSystemPositionOnly" type="system">
    <hardware>
      <plugin>ros2_control_demo_hardware/RRBotSystemPositionOnlyHardware</plugin>
      <param name="example_param_hw_start_duration_sec">2.0</param>
      <param name="example_param_hw_stop_duration_sec">3.0</param>
      <param name="example_param_hw_slowdown">2.0</param>
    </hardware>
    <joint name="joint1">
      <command_interface name="position">
        <param name="min">-1</param>
        <param name="max">1</param>
      </command_interface>
      <state_interface name="position"/>
    </joint>
    <joint name="joint2">
      <command_interface name="position">
        <param name="min">-1</param>
        <param name="max">1</param>
      </command_interface>
      <state_interface name="position"/>
    </joint>

    <transmission name="differential_transmission_1">
      <plugin>transmission_interface/DifferentialTransmission</plugin>
      <actuator name="joint1_motor">
        <role>actuator1</role>
      </actuator>
      <actuator name="joint2_motor">
        <role>actuator2</role>
      </actuator>
      <joint name="joint1">
        <role>joint1</role>
        <offset>0.5</offset>
        <mechanical_reduction>10</mechanical_reduction>
      </joint>
      <joint name="joint2">
        <role>joint2</role>
        <mechanical_reduction>50</mechanical_reduction>
      </joint>
    </transmission>
  </ros2_control>
```

### Proposal 2

Merging the current ros2_control xml format with the ROS1 transmissions but making `role` tags mandatory and one-liners:

```xml
  <ros2_control name="RRBotSystemPositionOnly" type="system">
    <hardware>
      <plugin>ros2_control_demo_hardware/RRBotSystemPositionOnlyHardware</plugin>
      <param name="example_param_hw_start_duration_sec">2.0</param>
      <param name="example_param_hw_stop_duration_sec">3.0</param>
      <param name="example_param_hw_slowdown">2.0</param>
    </hardware>
    <joint name="joint1">
      <command_interface name="position">
        <param name="min">-1</param>
        <param name="max">1</param>
      </command_interface>
      <state_interface name="position"/>
    </joint>
    <joint name="joint2">
      <command_interface name="position">
        <param name="min">-1</param>
        <param name="max">1</param>
      </command_interface>
      <state_interface name="position"/>
    </joint>

    <transmission name="differential_transmission_1">
      <plugin>transmission_interface/DifferentialTransmission</plugin>
      <actuator name="joint1_motor" role="actuator1"/>
      <actuator name="joint2_motor" role="actuator2"/>
      <joint name="joint1" role="joint1">
        <offset>0.5</offset>
        <mechanical_reduction>10</mechanical_reduction>
      </joint>
      <joint name="joint2" role="joint2">
        <mechanical_reduction>50</mechanical_reduction>
      </joint>
    </transmission>
  </ros2_control>
```



[simple_transmission]: images/simple_transmission.png "SimpleTransmission"
[four_bar_linkage_transmission]: images/four_bar_linkage_transmission.png "FourBarLinkageTransmission"

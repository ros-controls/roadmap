# ROS2 Control Components Architecture and URDF-Description Examples

## Architecture

![ros2_control Components Achitecture][ros2_control_arch_resource_manager]

[ros2_control_arch_resource_manager]: images/components_architecture.png "ROS2 Control - Components Architecture"

## URDF Examples

ros2_control implementation examples are presented for the following robot/robot-cell architectures:

1. Industrial Robots with only one interface
2. Industrial Robots with multiple interfaces (can not be written at the same time)
2.1. Robots with multiple interfaces used at the same time - the same structure as in (2)
3. Industrial Robots with a sensor integrated into the robot's control box
4. Industrial Robots with a sensor connected to ROS computer
5. Modular Robots with separate communication to each actuator
6. Modular Robots with actuators not providing states and additional sensors (simple Transmissions)
7. Modular Robots with separate communication to each "actuator" with multi joints (Transmission Example) - (system component is used)
8. Sensor only
9. Actuator Only

Note:
  * Everything within the `<classType>` tag is implemented as a plugin.
  * The examples below have some `<param>` tags defined. The names in those tags are primarily for demonstration, not part of a pre-defined XML schema. Each component may define their names inside the `<param>` tag.

#### 1. Industrial Robots with only one interface
  * the communication is done using proprietary API to communicate with robot control box
  * Data for all joints is exchanged in batch (at once)
  * Examples: KUKA RSI

```xml
  <ros2_control name="2DOFSystemRobotPositionOnly" type="system">
    <hardware>
      <classType>ros2_control_demo_hardware/2DOFSystemHardwarePositionOnly</classType>
      <param name="example_param_write_for_sec">2</param>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <joint name="joint1">
      <classType>ros2_control_components/PositionJoint</classType>
      <param name="min">-1</param>
      <param name="max">1</param>
    </joint>
    <joint name="joint2">
      <classType>ros2_control_components/PositionJoint</classType>
      <param name="min">-1</param>
      <param name="max">1</param>
    </joint>
  </ros2_control>
```

Note:
  * `ros2_control_components/PositionJoint`type has implicitly:
    ```xml
      <commandInterfaceType name="position"/>
      <stateInterfaceType name="position"/>
    ```

#### 2. Industrial Robots with multiple interfaces (can not be written at the same time)
  * the communication is done using proprietary API to communicate with robot control box
  * Data for all joints is exchanged in batch (at once)
  * Examples: KUKA FRI, ABB Yummy, Schunk LWA4p, etc.

```xml
  <ros2_control name="2DOFSystemRobotMultiInterface" type="system">
    <hardware>
      <classType>ros2_control_demo_hardware/2DOFSystemHardwareMultiInterface</classType>
      <param name="example_param_write_for_sec">2</param>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <joint name="joint1">
      <classType>ros2_control_components/MultiInterfaceJoint</classType>
      <commandInterfaceType name="position" min="-1" max="1"/>
      <commandInterfaceType name="velocity" min="-1" max="1"/>
      <commandInterfaceType name="effort" min="-0.5" max ="0.5"/>
      <stateInterfaceType name="position"/>
      <stateInterfaceType name="velocity"/>
      <stateInterfaceType name="effort"/>

    </joint>
    <joint name="joint2">
      <classType>ros2_control_components/MultiInterfaceJoint</classType>
      <commandInterfaceType name="position" min="-1" max="1"/>
      <stateInterfaceType name="position"/>
      <stateInterfaceType name="velocity"/>
      <stateInterfaceType name="effort"/>
    </joint>
  </ros2_control>
```

Note:
  * For `joint2` the "velocity" and "effort" command interfaces are intentionally left out to show another common use-case where only one interface can be commanded, but robot provides state of multiple types.


#### 2.1 Robots with multiple interfaces used at the same time - the same structure as in (2)
  * the communication is done using proprietary API to communicate with robot control box
  * Data for all joints is exchanged in batch (at once)
  * Multiple values can be commanded, e.g. goal position and the maximal velocity on the trajectory allowed
  * Examples: (humanoid robots?)

```xml
  <ros2_control name="2DOFSystemRobotMultiInterface" type="system">
    <hardware>
      <classType>ros2_control_demo_hardware/2DOFSystemHardwareMultiInterfaceMultiWrite</classType>
      <param name="example_param_write_for_sec">2</param>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <joint name="joint1">
      <classType>ros2_control_components/MultiInterfaceMultiWriteJoint</classType>
      <commandInterfaceType name="position" min="-1" max="1"/>
      <commandInterfaceType name="velocity" min="-1" max="1"/>
      <commandInterfaceType name="effort" min="-0.5" max ="0.5"/>
      <stateInterfaceType name="position"/>
      <stateInterfaceType name="velocity"/>
      <stateInterfaceType name="effort"/>
    </joint>
    <joint name="joint2">
      <classType>ros2_control_components/MultiInterfaceMultiWriteJoint</classType>
      <commandInterfaceType name="position" min="-1" max="1"/>
      <stateInterfaceType name="position"/>
      <stateInterfaceType name="velocity"/>
      <stateInterfaceType name="effort"/>
    </joint>
  </ros2_control>
```


#### 3. Industrial Robots with integrated sensor
  * the communication is done using proprietary API
  * Data for all joints is exchanged in batch (at once)
  * Sensor data are exchanged together with joint data
  * Examples: KUKA RSI with sensor connected to KRC (KUKA control box)

```xml
  <ros2_control name="2DOFSystemRobotwithSensor" type="system">
    <hardware>
      <classType>ros2_control_demo_hardware/2DOFSystemHardwareSensor</classType>
      <param name="example_param_write_for_sec">2</param>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <joint name="joint1">
      <classType>ros2_control_components/PositionJoint</classType>
      <param name="min">-1</param>
      <param name="max">1</param>
    </joint>
    <joint name="joint2">
      <classType>ros2_control_components/PositionJoint</classType>
      <param name="min">-1</param>
      <param name="max">1</param>
    </joint>
    <sensor name="tcp_fts_sensor">
      <classType>ros2_control_components/ForceTorqueSensor</classType>
      <param name="frame_id">kuka_tcp</param>
      <param name="lower_limits">-100</param>
      <param name="upper_limits">100</param>
    </sensor>
  </ros2_control>
```
Note:
  * `ros2_control_components/PositionJoint`type has implicitly:
    ```xml
      <commandInterfaceType name="position"/>
      <stateInterfaceType name="position"/>
    ```
    with the `min` and `max` parameters belonging to the `commandInterfaceType` element.


#### 4. Industrial Robots with externally connected sensor
  * the communication is done using proprietary API
  * Data for all joints is exchanged at once
  * Sensor data are exchanged independently
  * Examples: KUKA RSI and FTS connected to ROS-PC

```xml
  <ros2_control name="2DOFSystemRobotPositionOnlyExternalSensor" type="system">
    <hardware>
      <classType>ros2_control_demo_hardware/2DOFSystemHardwarePositionOnly</classType>
      <param name="example_param_write_for_sec">2</param>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <joint name="joint1">
      <classType>ros2_control_components/PositionJoint</classType>
      <param name="min">-1</param>
      <param name="max">1</param>
    </joint>
    <joint name="joint2">
      <classType>ros2_control_components/PositionJoint</classType>
      <param name="min">-1</param>
      <param name="max">1</param>
    </joint>
  </ros2_control>
  <ros2_control name="2DOFSystemRobotForceTorqueSensor" type="sensor">
    <hardware>
      <classType>ros2_control_demo_hardware/2D_Sensor_Force_Torque</classType>
      <param name="example_param_read_for_sec">0.43</param>
    </hardware>
    <sensor name="tcp_fts_sensor">
      <classType>ros2_control_components/ForceTorqueSensor</classType>
      <param name="frame_id">kuka_tcp</param>
      <param name="lower_limits">-100</param>
      <param name="upper_limits">100</param>
    </sensor>
  </ros2_control>
```

#### 5. Modular Robots with separate communication to each actuator
  * the communication is done on actuator level using proprietary or standardized API (e.g., canopen_402, Modbus, RS232, RS485)
  * Data for all actuators is exchanged separately from each other
  * Examples: Mara, Arduino-based-robots

```xml
  <ros2_control name="2DOFModularRobotJoint1"  type="actuator">
    <hardware>
      <classType>ros2_control_demo_hardware/PositionActuatorHadware</classType>
      <param name="example_param_write_for_sec">1.23</param>
      <param name="example_param_read_for_sec">3</param>
    </hardware>
    <joint name="joint1">
      <classType>ros2_control_components/PositionJoint</classType>
      <param name="min">-1</param>
      <param name="max">1</param>
    </joint>
  </ros2_control>
  <ros2_control name="2DOFModularRobotJoint2"  type="actuator">
    <hardware>
      <classType>ros2_control_demo_hardware/PositionActuatorHadware</classType>
      <param name="example_param_write_for_sec">1.23</param>
      <param name="example_param_read_for_sec">3</param>
    </hardware>
    <joint name="joint2">
      <classType>ros2_control_components/PositionJoint</classType>
      <param name="min">-1</param>
      <param name="max">1</param>
    </joint>
  </ros2_control>
```

#### 6. Modular Robots with actuators not providing states and with additional sensors (simple Transmissions)
  * the communication is done on actuator level using proprietary or standardized API (e.g., canopen_402, modbus, RS232, RS485)
  * Data for all actuators and sensors is exchanged separately from each other
  * Examples: Arduino-based-robots, custom robots

```xml
  <ros2_control name="2DOFModularRobotJoint1"  type="actuator">
    <hardware>
      <classType>ros2_control_demo_hardware/VelocityActuatorHadware</classType>
      <param name="example_param_write_for_sec">1.23</param>
      <param name="example_param_read_for_sec">3</param>
    </hardware>
    <joint name="joint1">
      <classType>ros2_control_components/VelocityJoint</classType>
      <commandInterfaceType name="velocity" min="-1" max="1"/>
    </joint>
    <transmission name="transmission1">
      <classType>transmission_interface/SimpleTansmission</classType>
      <param name="joint_to_actuator">${1024/PI}</param>
    </transmission>
  </ros2_control>
  <ros2_control name="2DOFModularRobotJoint2"  type="actuator">
    <hardware>
      <classType>ros2_control_demo_hardware/VelocityActuatorHadware</classType>
      <param name="example_param_write_for_sec">1.23</param>
      <param name="example_param_read_for_sec">3</param>
    </hardware>
    <joint name="joint2">
      <classType>ros2_control_components/VelocityJoint</classType>
      <commandInterfaceType name="velocity" min="-1" max="1"/>
    </joint>
  </ros2_control>
  <ros2_control name="2DOFSystemRobotPositionSensorJoint1" type="sensor">
    <hardware>
      <classType>ros2_control_demo_hardware/PositionSensorHardware</classType>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <joint name="joint1">
      <classType>ros2_control_components/PositionJoint</classType>
      <stateInterfaceType name="position"/>
    </joint>
  </ros2_control>
  <ros2_control name="2DOFSystemRobotPositionSensorJoint2" type="sensor">
    <hardware>
      <classType>ros2_control_demo_hardware/PositionSensorHardware</classType>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <joint name="joint2">
      <classType>ros2_control_components/PositionJoint</classType>
      <stateInterfaceType name="position"/>
    </joint>
  </ros2_control>
```
Note:
  * since there is only one keyword `commandInterfaceType` or `stateInterfaceType` given to `ros2_control_components/PositionJoint`type the other one is ignored for the hardware

#### 7. Modular Robots with separate communication to each "actuator" with multi joints (Transmission Example) - (system component is used)
  * the communication is done on actuator level using proprietary or standardized API (e.g., canopen_402)
  * Data for all actuators is exchanged separately from each other
  * There is a many-to-many connection between joint and actuator values resolved by a transmission
  * Examples: Wrist of a humanoid robot

```xml
  <ros2_control name="2DOFModularRobotWrist"  type="system">
    <hardware>
      <classType>ros2_control_demo_hardware/ActuatorHadwareMultiDOF</classType>
      <param name="example_param_write_for_sec">1.23</param>
      <param name="example_param_read_for_sec">3</param>
    </hardware>
    <joint name="joint1">
      <classType>ros2_control_components/PositionJoint</classType>
      <param name="min">-1</param>
      <param name="max">1</param>
    </joint>
    <joint name="joint2">
      <classType>ros2_control_components/PositionJoint</classType>
      <param name="min">-1</param>
      <param name="max">1</param>
    </joint>
    <transmission name="transmission1">
      <classType>transmission_interface/SomeComplex2by2Transmission</classType>
      <param name="joints">{joint1, joint2}</param>
      <param name="output">{output2, output2}</param>
      <param name="joint1_output1">1.5</param>
      <param name="joint1_output2">3.2</param>
      <param name="joint2_output1">3.1</param>
      <param name="joint2_output2">1.4</param>
    </transmission>
  </ros2_control>
```

#### 9. Sensor only
  * the communication is done on a sensor level using proprietary or standardized API
  * Examples: Camera, ForceTorqueSensor, Distance Sensors, IMU, etc.

```xml
  <ros2_control name="CameraWithIMU"  type="sensor">
    <hardware>
      <classType>ros2_control_demo_hardware/CameraWithIMUSensor</classType>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <sensor name="sensor1">
      <classType>ros2_control_components/IMUSensor</classType>
      <stateInterfaceType name="velocity"/>
      <stateInterfaceType name="acceleration"/>
    </sensor>
    <sensor name="sensor2">
      <classType>ros2_control_components/2DImageSensor</classType>
      <stateInterfaceType name="image"/>
    </sensor>
  </ros2_control>
```

#### 9. Actuator Only
  * the communication is done on actuator level using proprietary or standardized API
  * There may be a mechanical reduction or other type of 1-1 mapping between joint and actuator resolved by a transmission - there is no need to name the joint in the transmission tag
  * Examples: Small Conveyor, Motor, etc.

```xml
  <ros2_control name="2DOFModularRobotJoint1"  type="actuator">
    <hardware>
      <classType>ros2_control_demo_hardware/VelocityActuatorHadware</classType>
      <param name="example_param_write_for_sec">1.13</param>
      <param name="example_param_read_for_sec">3</param>
    </hardware>
    <joint name="joint1">
      <classType>ros2_control_components/VelocityJoint</classType>
      <param name="min">-1</param>
      <param name="max">1</param>
    </joint>
    <transmission name="transmission1">
      <classType>transmission_interface/RotationToLinerTansmission</classType>
      <param name="joint_to_actuator">${1024/PI}</param>
    </transmission>
  </ros2_control>
```

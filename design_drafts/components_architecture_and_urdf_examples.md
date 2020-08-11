# ROS2 Control Components Architecture and URDF-Description Examples

## Architecture

![ros2_control Components Achitecture][ros2_control_arch_resource_manager]

[ros2_control_arch_resource_manager]: images/components_architecture.png "ROS2 Control - Components Architecture"

## URDF Examples

ros2_control implementation examples are presented for the following robot/robot-cell architectures:

1. Industrial Robots with only one interface
2. Industrial Robots with multiple interfaces (can not be written at the same time)
3. Robots with multiple interfaces used at the same time
4. Industrial Robots with a sensor integrated into the robot's control box
5. Industrial Robots with a sensor connected to ROS computer
6. Modular Robots with separate communication to each actuator
7. Modular Robots with actuators not providing states and additional sensors (simple Transmissions)
8. Modular Robots with separate communication to each "actuator" with multi joints (Transmission Example) - (system component is used)
9. Sensor only
10. Actuator Only


#### 1. Industrial Robots with only one interface
  * the communication is done using proprietary API to communicate with robot control box
  * Data for all joints is exchanged in batch (at once)
  * Examples: KUKA RSI

```xml
  <ros2_control name="2DOF_System_Robot_Position_Only" type="system">
    <hardware>
      <classType>ros2_control_demo_hardware/2DOF_System_Hardware_Position_Only</classType>
      <param name="example_param_write_for_sec">2</param>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <joint name="joint1">
      <classType>ros2_control_components/PositionJoint</classType>
      <param name="min_values">-1</param>
      <param name="max_values">1</param>
    </joint>
    <joint name="joint2">
      <classType>ros2_control_components/PositionJoint</classType>
      <param name="min_values">-1</param>
      <param name="max_values">1</param>
    </joint>
  </ros2_control>
```

Note:
  * `ros2_control_components/PositionJoint`type has implicitly:
    ```xml
      <commandInterfaceType>position</commandInterfaceType>
      <stateInterfaceType>position</stateInterfaceType>
    ```

#### 2. Industrial Robots with multiple interfaces (can not be written at the same time)
  * the communication is done using proprietary API to communicate with robot control box
  * Data for all joints is exchanged in batch (at once)
  * Examples: KUKA FRI, ABB Yummy, Schunk LWA4p, etc.

```xml
  <ros2_control name="2DOF_System_Robot_MultiInterface" type="system">
    <hardware>
      <classType>ros2_control_demo_hardware/2DOF_System_Hardware_MultiInterface</classType>
      <param name="example_param_write_for_sec">2</param>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <joint name="joint1">
      <classType>ros2_control_components/MultiInterfaceJoint</classType>
      <commandInterfaceType>position</commandInterfaceType>
      <commandInterfaceType>velocity</commandInterfaceType>
      <commandInterfaceType>effort</commandInterfaceType>
      <stateInterfaceType>position</stateInterfaceType>
      <stateInterfaceType>velocity</stateInterfaceType>
      <stateInterfaceType>effort</stateInterfaceType>
      <param name="min_position_value">-1</param>
      <param name="max_position_value">1</param>
      <param name="min_velocity_value">-1</param>
      <param name="max_velocity_value">1</param>
      <param name="min_effort_value">-0.5</param>
      <param name="max_effort_value">0.5</param>
    </joint>
    <joint name="joint2">
      <classType>ros2_control_components/MultiInterfaceJoint</classType>
      <commandInterfaceType>position</commandInterfaceType>
      <stateInterfaceType>position</stateInterfaceType>
      <stateInterfaceType>velocity</stateInterfaceType>
      <stateInterfaceType>effort</stateInterfaceType>
      <param name="min_position_value">-1</param>
      <param name="max_position_value">1</param>
      <param name="min_velocity_value">-1</param>
      <param name="max_velocity_value">1</param>
      <param name="min_effort_value">-0.5</param>
      <param name="max_effort_value">0.5</param>
    </joint>
  </ros2_control>
```

Note:
  * For `joint2` the "velocity" and "effort" command interfaces are intentionally left out to show another often use-case


#### 3. Robots with multiple interfaces used at the same time
  * the communication is done using proprietary API or some standardized interface
  * Data for all joints is exchanged in batch (at once)
  * Examples: (some humanoid robots)?

```xml
  <ros2_control name="2DOF_System_Robot_MultiInterface_MultiWrite" type="system">
    <hardware>
      <classType>ros2_control_demo_hardware/2DOF_System_Hardware_MultiInterface_MultiWrite</classType>
      <param name="example_param_write_for_sec">2</param>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <joint name="joint1">
      <classType>ros2_control_components/MultiInterfaceJoint_MultiWrite</classType>
      <commandInterfaceType>position</commandInterfaceType>
      <commandInterfaceType>velocity</commandInterfaceType>
      <commandInterfaceType>effort</commandInterfaceType>
      <stateInterfaceType>position</stateInterfaceType>
      <stateInterfaceType>velocity</stateInterfaceType>
      <stateInterfaceType>effort</stateInterfaceType>
      <param name="min_position_value">-1</param>
      <param name="max_position_value">1</param>
      <param name="min_velocity_value">-1</param>
      <param name="max_velocity_value">1</param>
      <param name="min_effort_value">-0.5</param>
      <param name="max_effort_value">0.5</param>
    </joint>
    <joint name="joint2">
      <classType>ros2_control_components/MultiInterfaceJoint_MultiWrite</classType>
      <commandInterfaceType>position</commandInterfaceType>
      <commandInterfaceType>velocity</commandInterfaceType>
      <commandInterfaceType>effort</commandInterfaceType>
      <stateInterfaceType>position</stateInterfaceType>
      <stateInterfaceType>velocity</stateInterfaceType>
      <stateInterfaceType>effort</stateInterfaceType>
      <param name="min_position_value">-1</param>
      <param name="max_position_value">1</param>
      <param name="min_velocity_value">-1</param>
      <param name="max_velocity_value">1</param>
      <param name="min_effort_value">-0.5</param>
      <param name="max_effort_value">0.5</param>
    </joint>
  </ros2_control>
```

#### 4. Industrial Robots with a sensor integrated into the robot's control box
  * the communication is done using proprietary API
  * Data for all joints is exchanged in batch (at once)
  * Sensor data are exchanged together with joint data
  * Examples: KUKA RSI with sensor connected to KRC (KUKA control box)

```xml
  <ros2_control name="2DOF_System_Robot_with_Sensor" type="system">
    <hardware>
      <classType>ros2_control_demo_hardware/2DOF_System_Hardware_Sensor</classType>
      <param name="example_param_write_for_sec">2</param>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <joint name="joint1">
      <classType>ros2_control_components/PositionJoint</classType>
      <param name="can_read">True</param>
      <param name="min_values">-1</param>
      <param name="max_values">1</param>
    </joint>
    <joint name="joint2">
      <classType>ros2_control_components/PositionJoint</classType>
      <param name="can_read">True</param>
      <param name="min_values">-1</param>
      <param name="max_values">1</param>
    </joint>
    <sensor name="tcp_fts_sensor">
      <classType>ros2_control_components/ForceTorqueSensor</classType>
      <param name="frame_id">kuka_tcp</frame_id>
      <param name="lower_limits">-100</param>
      <param name="upper_limits">100</param>
    </sensor>
  </ros2_control>
```
Note:
  * `ros2_control_components/PositionJoint`type has implicitly:
    ```xml
      <commandInterfaceType>position</commandInterfaceType>
      <stateInterfaceType>position</stateInterfaceType>
    ```


#### 5. Industrial Robots with a sensor connected to ROS computer
  * the communication is done using proprietary API
  * Data for all joints is exchanged at once
  * Sensor data are exchanged independently
  * Examples: KUKA RSI and FTS connected to ROS-PC

```xml
  <ros2_control name="2DOF_System_Robot_Position_Only_External_Sensor" type="system">
    <hardware>
      <classType>ros2_control_demo_hardware/2DOF_System_Hardware_Position_Only</classType>
      <param name="example_param_write_for_sec">2</param>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <joint name="joint1">
      <classType>ros2_control_components/PositionJoint</classType>
      <param name="can_read">True</param>
      <param name="min_values">-1</param>
      <param name="max_values">1</param>
    </joint>
    <joint name="joint2">
      <classType>ros2_control_components/PositionJoint</classType>
      <param name="can_read">True</param>
      <param name="min_values">-1</param>
      <param name="max_values">1</param>
    </joint>
  </ros2_control>
  <ros2_control name="2DOF_System_Robot_ForceTorqueSensor" type="sensor">
    <hardware>
      <classType>ros2_control_demo_hardware/2D_Sensor_Force_Torque</classType>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <sensor name="tcp_fts_sensor">
      <classType>ros2_control_components/ForceTorqueSensor</classType>
      <param name="frame_id">kuka_tcp</frame_id>
      <param name="lower_limits">-100</param>
      <param name="upper_limits">100</param>
    </sensor>
  </ros2_control>
```

#### 6. Modular Robots with separate communication to each actuator
  * the communication is done on actuator level using proprietary or standardized API (e.g., canopen_402, Modbus, RS232, RS485)
  * Data for all actuators is exchanged separately from each other
  * Examples: Mara, Arduino-based-robots

```xml
  <ros2_control name="2DOF_Modular_Robot_joint1"  type="actuator">
    <hardware>
      <classType>ros2_control_demo_hardware/Position_Actuator_Hadware</classType>
      <param name="example_param_write_for_sec">1.23</param>
      <param name="example_param_read_for_sec">3</param>
    </hardware>
    <joint name="joint1">
      <classType>ros2_control_components/PositionJoint</classType>
      <param name="min_values">-1</param>
      <param name="max_values">1</param>
    </joint>
  </ros2_control>
  <ros2_control name="2DOF_Modular_Robot_joint2"  type="actuator">
    <hardware>
      <classType>ros2_control_demo_hardware/Position_Actuator_Hadware</classType>
      <param name="example_param_write_for_sec">1.23</param>
      <param name="example_param_read_for_sec">3</param>
    </hardware>
    <joint name="joint2">
      <classType>ros2_control_components/PositionJoint</classType>
      <param name="min_values">-1</param>
      <param name="max_values">1</param>
    </joint>
  </ros2_control>
```

#### 7. Modular Robots with actuators not providing states and additional sensors (simple Transmissions)
  * the communication is done on actuator level using proprietary or standardized API (e.g., canopen_402, modbus, RS232, RS485)
  * Data for all actuators and sensors is exchanged separately from each other
  * Examples: Arduino-based-robots, custom robots

```xml
  <ros2_control name="2DOF_Modular_Robot_joint1"  type="actuator">
    <hardware>
      <classType>ros2_control_demo_hardware/Velocity_Actuator_Hadware</classType>
      <param name="example_param_write_for_sec">1.23</param>
      <param name="example_param_read_for_sec">3</param>
    </hardware>
    <joint name="joint1">
      <classType>ros2_control_components/VelocityJoint</classType>
      <commandInterfaceType>position</commandInterfaceType>
      <param name="min_values">-1</param>
      <param name="max_values">1</param>
    </joint>
    <transmission name="transmission1">
      <type>transmission_interface/SimpleTansmission</type>
      <joint name="joint1">
        <interfaceType>velocity</interfaceType>
        <velocity_to_voltage>${1024/PI}</velocity_to_voltage>
      </joint>
    </transmission>
  </ros2_control>
  <ros2_control name="2DOF_Modular_Robot_joint2"  type="actuator">
    <hardware>
      <classType>ros2_control_demo_hardware/Velocity_Actuator_Hadware</classType>
      <param name="example_param_write_for_sec">1.23</param>
      <param name="example_param_read_for_sec">3</param>
    </hardware>
    <joint name="joint2">
      <classType>ros2_control_components/VelocityJoint</classType>
      <commandInterfaceType>position</commandInterfaceType>
      <param name="min_values">-1</param>
      <param name="max_values">1</param>
    </joint>
  </ros2_control>
  <ros2_control name="2DOF_System_Robot_Position_Sensor_joint1" type="sensor">
    <hardware>
      <classType>ros2_control_demo_hardware/Position_Sensor_Hardware</classType>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <joint name="joint1">
      <classType>ros2_control_components/PositionJoint</classType>
      <stateInterfaceType>position</stateInterfaceType>
      <param name="min_values">${-PI}</param>
      <param name="max_values">${PI}</param>
    </joint>
  </ros2_control>
  <ros2_control name="2DOF_System_Robot_Position_Sensor_joint2" type="sensor">
    <hardware>
      <classType>ros2_control_demo_hardware/Position_Sensor_Hardware</classType>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <joint name="joint2">
      <classType>ros2_control_components/PositionJoint</classType>
      <stateInterfaceType>position</stateInterfaceType>
      <param name="min_values">${-PI}</param>
      <param name="max_values">${PI}</param>
    </joint>
  </ros2_control>
```
Note:
  * since there is only one keyword `commandInterfaceType` or `stateInterfaceType` given to `ros2_control_components/PositionJoint`type the other one is ignored for the hardware

#### 8. Modular Robots with separate communication to each "actuator" with multi joints (Transmission Example) - (system component is used)
  * the communication is done on actuator level using proprietary or standardized API (e.g., canopen_402)
  * Data for all actuator is exchanged separately from each other
  * Examples: Wrist of a humanoid robot

```xml
  <ros2_control name="2DOF_Modular_Robot_Wrist"  type="system">
    <hardware>
      <classType>ros2_control_demo_hardware/Actuator_Hadware_MultiDOF</classType>
      <param name="example_param_write_for_sec">1.23</param>
      <param name="example_param_read_for_sec">3</param>
    </hardware>
    <joint name="joint1">
      <classType>ros2_control_components/PositionJoint</classType>
      <param name="can_read">True</param>
      <param name="min_values">-1</param>
      <param name="max_values">1</param>
    </joint>
    <joint name="joint2">
      <classType>ros2_control_components/PositionJoint</classType>
      <param name="can_read">True</param>
      <param name="min_values">-1</param>
      <param name="max_values">1</param>
    </joint>
    <transmission name="transmission1">
      <type>transmission_interface/SomeComplex_2x2_Transmission</type>
      <joint name="joint1">
        <interfaceType>position</interfaceType>
        <mechanicalReduction_output1>1.5</mechanicalReduction_output1>
        <mechanicalReduction_output2>3.2</mechanicalReduction_output2>
      </joint>
      <joint name="joint2">
        <interfaceType>position</interfaceType>
        <mechanicalReduction_output1>3.1</mechanicalReduction_output1>
        <mechanicalReduction_output2>1.4</mechanicalReduction_output2>
      </joint>
    </transmission>
  </ros2_control>
```

#### 9. Sensor only
  * the communication is done on a sensor level using proprietary or standardized API
  * Examples: Camera, ForceTorqueSensor, Distance Sensors, IMU, etc.

```xml
  <ros2_control name="Camera_with_IMU"  type="sensor">
    <hardware>
      <classType>ros2_control_demo_hardware/CameraWithIMU_Sensor</classType>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <sensor name="sensor1">
      <classType>ros2_control_components/IMUSensor</classType>
      <interfaceType>velocity</interfaceType>
      <interfaceType>acceleration</interfaceType>
      <param name="min_values">-1000</param>
      <param name="max_values">1000</param>
    </joint>
    <joint name="sensor2">
      <classType>ros2_control_components/2DImageSensor</classType>
      <interfaceType>image</interfaceType>
      <param name="min_values">0</param>
      <param name="max_values">255</param>
    </joint>
  </ros2_control>
```

#### 10. Actuator Only
  * the communication is done on actuator level using proprietary or standardized API
  * Examples: Small Conveyor, Motor, etc.

```xml
  <ros2_control name="2DOF_Modular_Robot_joint1"  type="actuator">
    <hardware>
      <classType>ros2_control_demo_hardware/Velocity_Actuator_Hadware</classType>
      <param name="example_param_write_for_sec">1.23</param>
      <param name="example_param_read_for_sec">3</param>
    </hardware>
    <joint name="joint1">
      <classType>ros2_control_components/VelocityJoint</classType>
      <param name="min_values">-1</param>
      <param name="max_values">1</param>
    </joint>
    <transmission name="transmission1">
      <type>transmission_interface/RotationToLinerTansmission</type>
      <joint name="joint1">
        <interfaceType>velocity</interfaceType>
        <rotation_to_linenar_velocity>${2*radius*PI}</rotation_to_linenar_velocity>
      </joint>
    </transmission>
  </ros2_control>
```

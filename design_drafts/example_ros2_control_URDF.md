# ROS2 Control Components Architecture and URDFs

## Architecture

![ros2_control Components Achitecture][ros2_control_arch_ressource_manager]

[ros2_control_arch_ressource_manager]: images/resource_manger_ds.svg "ROS2 Control - Components Architecture"

## URDF Examples

ros2_control implements following robot/robot-cell architectures:

1. Industrial Robots with only one interface
2. Industrial Robots with multiple interfaces used at the same time
3. Industrial Robots with multiple interfaces (can not be used at the same time)
4. Industrial Robots with sensor integrated in the robot controller
5. Industrial Robots with sensor connected to ROS computer 
6. Modular Robots with separate communication to each actor
7. Modular Robots with separate communication to each actor Multi joints (Transmissions
8.  Example)
8. Sensor only

#### 1. Industrial Robots with only one interface
  * the communication is done using proprietary API
  * Data for all joints is exchanged at once
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
```

#### 2. Industrial Robots with multiple interfaces used at the same time
  * the communication is done using proprietary API
  * Data for all joints is exchanged at once
  * Examples: KUKA FRI, ABB Yummy, etc.

```xml
  <ros2_control name="2DOF_System_Robot_MultiInterface" type="system">
    <hardware>
      <classType>ros2_control_demo_hardware/2DOF_System_Hardware_MultiInterface</classType>
      <param name="example_param_write_for_sec">2</param>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <joint name="joint1">
      <classType>ros2_control_components/GenericJoint</classType>
      <interfaceType>position</interfaceType>
      <interfaceType>velocity</interfaceType>
      <interfaceType>effort</interfaceType>
      <param name="multi_interface">True</param>
      <param name="can_read">True</param>
      <param name="min_values">-1</param>
      <param name="max_values">1</param>
    </joint>
    <joint name="joint2">
      <classType>ros2_control_components/GenericJoint</classType>
      <interfaceType>position</interfaceType>
      <interfaceType>effort</interfaceType>   <!-- "velocity" intentionally left out" -->     
      <param name="multi_interface">True</param>
      <param name="can_read">True</param>
      <param name="min_values">-1</param>
      <param name="max_values">1</param>
    </joint>
  </ros2_control>
```

#### 3. Industrial Robots with multiple interfaces (can not be used at the same time)
  * the communication is done using proprietary API
  * Data for all joints is exchanged at once
  * Examples: Schunk LWA4p

```xml
  <ros2_control name="2DOF_System_Robot_MultiInterface_Limited" type="system">
    <hardware>
      <classType>ros2_control_demo_hardware/2DOF_System_Hardware_MultiInterface_Limited</classType>
      <param name="example_param_write_for_sec">2</param>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <joint name="joint1">
      <classType>ros2_control_components/GenericJoint</classType>
      <interfaceType>position</interfaceType>
      <interfaceType>velocity</interfaceType>
      <param name="multi_interface">False</param>
      <param name="can_read">True</param>
      <param name="min_values">-1</param>
      <param name="max_values">1</param>
    </joint>
    <joint name="joint2">
      <classType>ros2_control_components/GenericJoint</classType>
      <interfaceType>position</interfaceType>
      <interfaceType>velocity</interfaceType>    
      <param name="multi_interface">False</param>
      <param name="can_read">True</param>
      <param name="min_values">-1</param>
      <param name="max_values">1</param>
    </joint>
    </joint>
  </ros2_control>
```

#### 4. Industrial Robots with sensor integrated in the robot controller
  * the communication is done using proprietary API
  * Data for all joints is exchanged at once
  * Sensor data are exchanged together with joint data
  * Examples: KUKA RSI with sensor connected to KRC

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
      <param name="min_values">-5</param>
      <param name="max_values">1</param>
    </sensor>
  </ros2_control>
```

#### 5. Industrial Robots with sensor connected to ROS computer 
  * the communication is done using proprietary API
  * Data for all joints is exchanged at once
  * Sensor data are exchanged independently
  * Examples: KUKA RSI and FTS connected to ROS-PC

```xml
  <ros2_control name="2DOF_System_Robot_Position_Only" type="system">
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
  </ros2_control>
  <ros2_control name="2DOF_System_Robot_Position_Only" type="sensor">
    <hardware>
      <classType>ros2_control_demo_hardware/2D_Sensor_Force_Torque</classType>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <sensor name="tcp_fts_sensor">
      <classType>ros2_control_components/ForceTorqueSensor</classType>
      <param name="frame_id">kuka_tcp</frame_id>
      <param name="min_values">-5</param>
      <param name="max_values">1</param>
    </sensor>
  </ros2_control>
```

#### 6. Modular Robots with separate communication to each actor
  * the communication is done on actuator level using proprietary or standardized API (e.g. canopen_402)
  * Data for all actuators is exchanged separately from each other
  * Examples: Mara, Schunk LWA4p (it is possible by splitting ros_canopen)

```xml
  <ros2_control name="2DOF_Modular_Robot_joint1"  type="actuator">
    <hardware>
      <classType>ros2_control_demo_hardware/Actuator_Hadware_1DOF</classType>
      <param name="example_param_write_for_sec">1.23</param>
      <param name="example_param_read_for_sec">3</param>
    </hardware>
    <joint name="joint1">
      <classType>ros2_control_components/PositionJoint</classType>
      <param name="can_read">True</param>
      <param name="min_values">-1</param>
      <param name="max_values">1</param>
    </joint>
  </ros2_control>  
  <ros2_control name="2DOF_Modular_Robot_joint2"  type="actuator">
    <hardware>
      <classType>ros2_control_demo_hardware/Actuator_Hadware_1DOF</classType>
      <param name="example_param_write_for_sec">1.23</param>
      <param name="example_param_read_for_sec">3</param>
    </hardware>
    <joint name="joint2">
      <classType>ros2_control_components/PositionJoint</classType>
      <param name="can_read">True</param>
      <param name="min_values">-1</param>
      <param name="max_values">1</param>
    </joint>
  </ros2_control>
```

#### 7. Modular Robots with separate communication to each actor Multi joints (Transmission Example)
  * the communication is done on actuator level using proprietary or standardized API (e.g. canopen_402)
  * Data for all actuator is exchanged separately from each other
  * Examples: Wrist of a humanoid robot

```xml
  <ros2_control name="2DOF_Modular_Robot_Wrist"  type="actuator">
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
    <transmission name="tran1">
      <type>transmission_interface/SomeComplex_2x2_Transmission</type>
      <joint name="joint1">
        <interfaceType>positon</interfaceType>
      </joint>
      <joint name="joint2">
        <interfaceType>positon</interfaceType>
      </joint>
      <actuator name="motor1">
        <mechanicalReduction>1</mechanicalReduction>
      </actuator>
      <actuator name="motor2">
        <mechanicalReduction>1</mechanicalReduction>
      </actuator>
    </transmission>
  </ros2_control>
```

#### 8. Sensor only
  * the communication is done on sensor level using proprietary or standardized API
  * Examples: Camera, ForceTorqueSensor, Distance Sensors, IMU etc..

```xml
  <ros2_control name="Camera_with_IMU"  type="sensor">
    <hardware>
      <classType>ros2_control_demo_hardware/Generic_Sensor_MultiDOF</classType>
      <param name="example_param_read_for_sec">2</param>
    </hardware>
    <sensor name="sensor1">
      <classType>ros2_control_components/GenericSensor</classType>
      <interfaceType>velocity</interfaceType>
      <interfaceType>acceleration</interfaceType>
      <param name="min_values">-1</param>
      <param name="max_values">1</param>
    </joint>
    <joint name="sensor2">
      <classType>ros2_control_components/GenericSensor</classType>
      <interfaceType>image</interfaceType>
      <param name="min_values">0</param>
      <param name="max_values">255</param>
    </joint>
  </ros2_control>
```

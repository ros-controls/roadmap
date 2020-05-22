# Structure of Hardware Interfaces

The following shall discuss the design of data structures in the `hardware_interface` package for ROS2.
The document deals only with hardware description, but this also depends on [controllers execution management design](controller_execution_management.md).
As of today (ROS1), a `robot` is a fundamental and rigid structure that handles any hardware.
Therefore it is not possible to extend it with additional hardware, like sensors, actuators, and tools, without coding.
This design tries to achieve the following:

* Consistent naming of classes with the wording used in control theory (should simplify starting for new users);
* Logical implementation of relations between robots, sensors, actuators and finally controllers (should simplify starting for new users);
* Easy extension of a robot with additional hardware (no need for compilation).


## Motivation

The `RobotHW` class in ROS1 is the basic structure for representing any hardware, robots, sensors, and actuators.
From the control theory perspective, a robot is an assembly with one or more sensors and actuators.
Therefore we should strive such a design, primarily because it provides a few pleasing side effects:

* Simple extension with additional hardware (e.g., extending an industrial robot with an additional sensor on the TCP );
* Dynamic extension of a robot with other equipment (e.g., tool changers);
* Reuse of hardware definitions and interfaces without a need for coding and compiling of robot hardware.

## Nomenclature

### Robot Hardware

A robot is logically represented with the `RobotHardware` class.
This structure has at least one sensor and one actuator, but it generally represents any composition of `RobotHardware`, `SensorHardware`, and `ActuatorHardware` structures.
Specific implementation takes care to read/write data to physical hardware properly.
A specific implementation of the `RobotHardware` decides if the data are received/sent at once (e.g., industrial robot). Its sensors and actuators can read/write from/to physical hardware (e.g., a sensor attached to a robot's TCP).

### Sensor Hardware

A sensor in a robotic system is represented with the `SensorHardware` class.
Only data reading is possible from this type of hardware.
Therefore no resource conflict is possible, and it will not be checked.

### Actuator Hardware

An actuator in a robotic system is represented with the `ActuatorHardware` class.
Only data writing is possible for this type of hardware.
Resource conflict should be strictly checked, managed, and protected (e.g., by a key provided by a specific controller).

## Package and Class Structure

The following subsections describe the internal class structure of `ros2_control`.
The figure at the end of this section gives a complete overview with examples.

This structure has the following intentions:

1. `Robot` class should not be the basic class anymore since we also want to integrate simpler structures like sensors and 1 DoF actuators.
1. Internal data structures should be agnostic of specific interface types and data this interface manages. An important part for this is [Flexible Joint State Message](flexible_joint_states_msg.md) data type.
1. Users should be able to use Sensors and Actuators directly as well as bundle them to a robot.
1. The data structure should enable management of communication with a robot at once (e.g., KUKA RSI communication) and also separating it into multiple `Sensors` and `Actuators` (`Joints`) (e.g., Schunk LWA4p which uses ros_canopen) where each of them has a separate communication channel.

The classes are separated in following logical packages:

1. `ros2_control_components` provides a digital representation of components used in a robotic system.
1. `ros2_control_hardware_interface` provides "connection" between digital model from `ros2_control_components` and concrete communication interface to a real hardware.
1. `ros2_control_communication_interfaces` defines structures of specific communication interfaces to enable simpler integration for the end-users.

### `ros2_control_components`

The package `ros2_control_components` models hardware components of which are used by the `ros2_control` framework.
These classes are used for storing run-time data and for access from controllers.
All classes in this package implement the `BaseComponent` interface.
The primary purpose of this component is to separate the inheritance of `Robot` class versus `Sensor` and `Actuator` classes.
This interface keeps references to the `ComponentHardwareInterface` interface and provides some basic functionality needed by all components.

The `Component` class enables storing of values and necessary information like `frame_id`.
The end-user should not use this class.
The classes `Sensor` and `Actuator` are first level classes that can be used by a user.
They define only a basic structure and should be extended for a specific type of a sensor (e.g., ForceTorqueSensor class) and actuator (e.g., PositionActuator).
The `Robot` class is the complex class that holds references for its `Sensor`, `Actuator`, and other `Robot` classes in a case of combined robot hardware (e.g., mobile manipulator).


### `ros2_control_hardware_interface`

The package `ros2_control_hardware_interface` serves as a connection between virtual representation of robotic hardware in the package `ros2_control_components` with the specific communication interfaces.
Therefore, this package provides equivalent classes to those in `ros2_control_components`, but defines what functionalities internal model is expecting from specific hardware.

### `ros2_control_communication_interfaces`

The package `ros2_control_communication_interfaces` defines some standard communication interfaces used for control of the robots to cut the integration time for the end-users.

### Class Diagram

The following class diagram shows the internal structures of `ros2_control`-hardware interface.
In blue are marked example components used in ROS1.
Green color marks the structure relevant for the example.
Red components are needed in a case if one needs some spatial robot abstraction for hardware (they should probably be deleted.

![ROS2 Control Class Diagram][ros2_control_core_diagram]

## Using `ros2_control` With a new Robot

The process of making a new robot `ros2_control` compatible is depicted in the flow chart hereunder.
One needs to do the following steps:

1. Check if needed `HardwareCommunicationInterface` is already integrated into `ros2_control`, and if not, it is recommended to do it.
For this, get in touch with people responsible for the `ros2_control_hardware_comunication` package.
2. Check if needed `HardwareInterface`, i.e., logical part of the communication with the robot, exists and if not implement it.
This class has to inherit the `ComponentHardwareInterface` interface.
3. Check if all types of `Sensor` and `Actuator` classes exist in `ros2_control_components` (or some third-party repository) and if not get in touch with people responsible for the repository and decide where to implement it.
4. Check `ros2_control_demo` repository, for example, files and templates to create YAMLs, URDF, and launch files for your robot.

![ROS2 Control - Enabling a new Robot][ros2_control_new_robot]


## Example use-case

The reasoning behind this structure is based on the abstraction of functionality and hardware access for multiple kinds of robots used with and without additional sensors.
To extend the robot with additional actuators follows the same logic.

For better understanding, please consider the following use-case:
* In a ROS-based factory, there is a process which needs force-controlled robots.
* Following hardware is provided:
  * Robot1 with "batch" interface for communication (e.g., KUKA robots with RSI);
  * Robot2 with an interface where each joint can be addressed separately (e.g., Schunk LWA4p with canopen);
  * Sensor1 with a proprietary protocol for communication (e.g., ATI Force-Torque Sensors with CAN interface);
  * Sensor2 with a different proprietary protocol for communication (e.g., Schunk FTC50 sensor with CAN interface).
  
* The Robot1 (KUKARobot) is then used as follows:
  * A new class `KUKA_RSI_HardwareInterface` implementing RSI communication protocol for KUKA robots and extending `TCP/IP_HWCommunicaitonInterface` is written.
  * To start the robot, a YAML configuration for `Robot` class needs to be written.
  This file would look something like:
  ```
    RobotKUKA:
      name: "KUKA KR5 arc"
      RobotHardware:
        type: "kuka_driver/KUKA_RSI_HardwareInterface"
        HardwareCommunicationInterface:
          rsi_type: 4  #RSI-Fast with 4ms loop
          interface: eth_kuka
          interface_ip: 192.168.1.1

      # Parameters for Robot class
      joints: [joint1, joint2, joint3, joint4, joint5, joint6]
      actuators:
        joint1:
          name: joint1
          type: "ros2_control_components/PositionActuator"
          n_dof: 1
          max_values: [PI]
          min_values: [-PI]
        joint2:
          ...
      sensors:
        joint1:
          name: joint1
          type: "ros2_control_components/PositionSensor"
        ...
  ```
  
* The Robot2 (Schunk) is then used as follows:
  * To use this robot, one can reuse `canopen_motor_HWCommunicationInterface` implemented for communication with canopen motors (Profile 402).
  * Therefore, the YAML configuration of `Robot` class is a bit longer.
  This file would look something like:
  ```
    RobotSchunk:
      name: "Schunk LWA4p"

      # Parameters for Robot class
      joints: [joint1, joint2, joint3, joint4, joint5, joint6]
      actuators:
        joint1:
          name: joint1
          type: "ros2_control_components/PositionActuator"
          n_dof: 1
          max_values: [PI]
          min_values: [-PI]
          ActuatorHW:
            type: "ros_canopen/canopen_motor_HWCommunicationInterface"
            interface_path: can0
            socket: true
            can_id: 3
            profile: 402
            eds_file: "schunk_lwa4p/config/Schunk_0_63.dcf"
        joint2:
          ...
      sensors:
        joint1:
          name: joint1
          type: "ros2_control_components/PositionSensor"
        ...
  ```
  
* The Sensor1 (ATI FTS with CAN) is integrated as follows:
  * A new class `ATI_ForceTorqueSensorHW_CAN` which extends `ForceTorqueSensorHW` is written.
  This class implements ATI's communication protocol, e.g., fetching the calibration matrices, calculation of FT-data from strain gauge values, etc.
  This class uses `Generic_CAN_HWCommunicationInterface` to be able to dynamically load specific CAN-device library depending on the manufacturer of used CAN adapter (e.g., PEAK, ESD, ...).
  * To use this class by ROS, a YAML configuration for `ForceTorqueSensor` class, which provides high-level functions (e.g., noise filters, offset, and gravity compensation), needs to be written.
  This file would look something like:
  ```
  SensorATI:
    name: "ATI Force Torque Sensor"
    SensorHardware:
      type: "ati_force_torque_sensors/ATI_ForceTorqueSensorHW_CAN"
      data_pull_frequency: 1000 #Hz
      HardwareCommunicationInterface:
        type: "PEAK_CAN_HWCommunicatonInterface"
        path: can0
        base_id: 0x20

    # Parameters for ForceTorqueSensor class  
    data_publish_frequency: 200 #Hz
    sensor_frame: "fts_reference_link"
    robot_base_frame: "robot_base_link"
    auto_init: true
    offset:
      is_static: false #Caculate always sensor offen on start
    filters:
      - MovingMean:
        - device: 3
      - ThresholdFilter:
        - linear:
          - x: 2
    ...
  ```

* The Sensor2 (Schunk FTC50 with CAN) is integrated as follows:
  * A new class `Schunk FTC50_ForceTorqueSensorHW_CAN` which extends `ForceTorqueSensorHW` is written.
  * Other steps are the same as for Sensor1.
  This first part of the configuration would change to something like:
  ```
  SensorSchunk:
    name: "Schunk Force Torque Sensor"
    SensorHardware: 
      type: "schunk_force_torque/FTC50_ForceTorqueSensorHW_CAN"
  ...
  ```
 
Now there is a possibility to combine the hardware as needed:
  * The example file for Robot1 (KUKARobot) and Sensor2 (Schunk FTC50 with CAN):
  ```
    RobotSensorKUKASchunk:
      name: "KUKA KR5 arc with Schunk FTC50 on TCP"
      robots:
        RobotKUKA:
          # Description from above
      
      sensors:
        SensorSchunk:
          # Description from above
  ```

### Class Diagram of the Example

![ROS2 Control Class Diagram][ros2_control_example_diagram]


<!-- List of References -->
[ros2_control_core_diagram]: images/ros2_control_core_diagram.svg "ROS2 Control - Class Diagram"
[ros2_control_example_diagram]: images/ros2_control_example_diagram.svg "ROS2 Control - Example"
[ros2_control_new_robot]: images/ros2_control_new_robot.svg "ROS2 Control - Enabling a new Robot"
[controllers execution mangagemnt design]:https://github.com/Karsten1987/roadmap/blob/controller_execution_management/design_drafts/controller_execution_management.md

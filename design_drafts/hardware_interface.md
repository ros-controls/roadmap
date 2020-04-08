# Structure of Hardware Interfaces

The following shall discuss the design of data structures in the `hardware_interface` package for ROS2.
The document deals only with hardware description, but this also depends on [controllers execution management design].
As of today (ROS1), a `robot` is a fundamental and rigid structure that handles any hardware.
Therefore to extend it with additional hardware, like sensors, actuators, and tools, without coding and recompiling it.
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

A sensor in a robotic system is represented with the `SesorHardware` class.
Only data reading is possible from this type of hardware.
Therefore no resource conflict is possible, and it will not be checked.

### Actuator Hardware

An actuator in a robotic system is represented with the `ActuatorHardware` class.
Only data writing is possible for this type of hardware.
Resource conflict should be strictly checked, managed, and protected (e.g., by a key provided by a specific controller).


## Class Structure

The details on internal class strucutre of `ros2_control` is described in follwing subsections.
A complete overview is given in the figure on the end of this section.

This structure has following intentions:

1. `Robot` class should not be basic class any more, since we want also integrate simpler strucutres like sensors and 1 DoF actuators.
1. Internal data structures should be agnostic of specific interface type and data this interface manges. An important part for this is [Flexible Joint State Message](flexible_joint_states_msg.md) data type.
1. Users should be able to use Sensor and Actuators directly as well as bundelded them to a robot.
1. The data structure should enable management of communication with a robot at once (e.g. KUKA RSI communication) and also separating it into multiple `Sensors` and `Actuators` (`Joints`) (e.g. Schunk LWA4p which uses ros_canopen) where each of them has separate communication channel.

The classes are separated in following logical packages:
1. `ros2_control_components` provides digital representation of components used in a robotic system.
1. `ros2_control_hardware_interface` provides "connection" between digital model from `ros2_control_components` and concete communication interface to a real hardware.
1. `ros2_control_communication_interfaces` provides definition for structures of specific communication interfaces to enable simpler integration for the end users.

### `ros2_control_components`

The package `ros2_control_components` models hardware components of which are used by `ros2_control` framework.
The classes are uses for storing run-time data and for acces from controllers.
The `BaseComponent` class which all members of this package should extend and its main purpose is to keep logical order of 
The `Component` class enalbes storing of values and basic informations like `frame_id`.
It is not intended to be used by a user.
The classes `Sensor` and `Actuator` are first level classes which can be used by a user.
They define only a basic structure and should be extended for specific type of a sensor (e.g. ForceTorqueSensor class) and actuator (e.g. PositionActuator).
The `Robot` class is complex class which holds references for its `Sensor`, `Actuator` and other `Robot` classes in a case of combined robot hardware (e.g. mobile manipulator).


### `ros2_control_hardware_interface`

The package `ros2_control_hardware_interface` servs as connection between virtual repsentation of robotic hardware in the package `ros2_control_components` with the specific communication interfaces.
Therefore, this package provides equivalent classes to those in `ros2_control_components`, but defines what functinalities is internal model is exepcting from a specific hardware.

### `ros2_control_communication_interfaces`

The package `ros2_control_communication_interfaces` defines some standard communication interfaces used for control of the robots to cut the integration time for the end-users.
**Note:** This is maybe out of scope?


### Example use-case of for the classes

The reasonong behind this structure is based on abstraction of functionalty and hardware access for multiple kind of robots used with and without additional sensors.
To extend the robot with additional actuators follows the same logic.
For better understanding please consider following use-case:
* In a ROS-runned factory there is a process which need force controlled robots.
* Following hardware is provided:
  * Robot1 with "batch" interface for communication" (e.g. KUKA robots with RSI);
  * Robot2 with interface where each joint can be addressed separately (e.g. Schunk LWA4p with canopen);
  * Sensor1 with propiatery protocol for communication (e.g. ATI Force-Torque Sensors with CAN interface);
  * Sensor2 with different propriatrey protocol for communication (e.g. Schunk FTC50 sensor with CAN interface).
  
* The Robot1 (KUKARobot) is then used as follows:
  * A new class `KUKARobot` is written which extends 
  
  
* The Sensor1 (ATI FTS with CAN) is integrated as follows:
  * A new class `ATI_ForceTorqueSensorHW_CAN` which extends `ForceTorqueSensorHW` is written.
  This class implements ATI's communication protocol, e.g. fetching the calibration matrices, calculation of FT-data from strain gaugle values, etc.
  This class uses `Generic_CAN_HWCommunicationInterface` to be able to dynamically load specific CAN-device library depending on the manufacturer of used CAN interface (e.g. PEAK, ESD, ...).
  * To use this class by ROS, a yaml configuration for `ForceTorqueSensor` class which provides high-level functions (e.g. noise filters, offset and gravity compensation) needs to be written.
  This file would look someting like:
  ```
  SensorHW:
    name: "ATI Force Torque Sensor"
    type: "ati_force_torque_sensors/ATI_ForceTorqueSensorHW_CAN"
    data_pull_frequency: 1000 #Hz
    HWCommunicationInterface:
      type: "PEAK_CAN_HWCommunicatonInterface"
      path: can0
      base_id: 0x20
    
  # Parameters for   
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



To help integration of new robots into `ros2_control` following is proposed:

The following class diagram shows interal structures of `ros2_control`-hardware interface:

![ros2_control_hardware_interfaces](images/ros2_control_hardware_interfaces.png "ROS2 Control - Hardware Interfaces")



<!-- List of References -->
[controllers execution mangagemnt design]:https://github.com/Karsten1987/roadmap/blob/controller_execution_management/design_drafts/controller_execution_management.md

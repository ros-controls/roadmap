===========================
ros2_control Framework
===========================

The ros2_control is a framework for (real-time) control of robots using `ROS` (`Robot Operating System <http://ros.org>`__).
Its packages are a rewrite of `ros_control <http://wiki.ros.org/ros_control>`__ packages to simplify integrating new hardware and overcome some drawbacks.

If you are not familiar with the control theory, please get some idea about it (e.g., at `Wikipedia <https://en.wikipedia.org/wiki/Control_theory>`_) to get familiar with the terms used in this manual.

.. contents:: Table of Contents
   :depth: 2
   
Overview
========
The ros2_control framework's source can be found in `ros-controlls/ros2_control`_ and `ros-controls/ros2_controllers`_ GitHub-repositories.
The following figure shows the Architecture of the ros2_control framework.

|ros2_control_architecture|

Controller Manager
------------------
The `Controller Manager`_ (CM) connects the controllers' and hardware-abstraction sides of the ros2_control framework.
It also serves as the entry-point for users through ROS services.
The CM implements a node without an executor so it can be integrated into a custom setup.
Still, for a standard user, it is recommended to use the default node-setup implemented in `ros2_control_node <https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/ros2_control_node.cpp>`_ file from the ``controller_manager`` package.
This manual assumes that you use this default node-setup.

On the one side, CM manages (e.g., loading, activating, deactivating, unloading) controllers and from them required interfaces.
On the other side, it has access to the hardware components (through Resource Manager), i.e., their interfaces.
The Controller Manager matches *required* and *provided* interfaces, gives controllers access to hardware when activated, or reports an error if there is an access conflict.

The execution of the control-loop is managed by the CM's ``update()`` method.
The method reads data from the hardware components, updates outputs of all active controllers, and writes the result to the components.

Resource Manager
----------------
The `Resource Manager`_ (RM) abstracts physical hardware and its drivers (called *hardware components*) for the ros2_control framework.
The RM loads the components using ``pluginlib``-library, manages their lifecycle and components' state and command interfaces.
This abstraction provided by RM enables re-usability of implemented hardware components, e.g., robot and gripper, without any implementation and flexible hardware application for state and command interfaces, e.g., separate hardware/communication libraries for motor control and encoder reading.

In the control loop execution, the RM's ``read()`` and ``write()`` methods deal with communication to the hardware components.

.. _overview-controllers:
Controllers
-----------
The controllers in the ros2_control framework have the same functionality as defined in the control theory. They compare the reference value with the measured output and, based on this error, calculate a system's input (for more details, visit `Wikipedia <https://en.wikipedia.org/wiki/Control_theory>`_).
The controlles are objects derived from `ControllerInterface`_ (``controller_interface`` package in `ros-controls/ros2_control`_) and exported as plugins using ``pluginlib``-library.
For example of on controller check `ForwardCommandController implementation`_ in the `ros-controls/ros2_controllers`_ repository.
The controllers' lifecycle is based on the `LifecycleNode-Class`_ implementing the state machine as described in the `Node Lifecycle Design`_ document.

When executing the control-loop ``update()`` method is called.
The method can access the latest hardware states and enables the controller to write the hardware's command interfaces.

User Interfaces
---------------
Users interact with the ros2_control framework using `Controller Manager`_'s services.
Those can be used directly or through the "cli"-interface (base command: ``ros2 control``).
The "cli"-interface is more user-friendly for direct interaction, i.e., outside of a node.

For a list of services and their definitions, check the ``srv`` folder in the `controller_manager_msgs`_ package.

For the description of the "cli"-interface, see the ``README.md`` file of the `ros2controlcli`_ package.


Hardware Components
===================
The *hardware components* realize communication to physical hardware and represent its abstraction in the ros2_control framework.
The components have to be exported as plugins using ``pluginlib``-library.
The `Resource Manager`_ dynamically loads those plugins and manages their lifecycle.

There are three basic types of components:

System
  Complex (multi-DOF) robotic hardware like industrial robots.
  The main difference between the *Actuator* component is the possibility to use complex transmissions like needed for humanoid robot's hands.
  This component has reading and writing capabilities.
  It is used when the is only one logical communication channel to the hardware (e.g., KUKA-RSI).
  
Sensor
  Robotic hardware is used for sensing its environment.
  A sensor component is related to a joint (e.g., encoder) or a link (e.g., force-torque sensor).
  This component type has only reading capabilities.

Actuator
  Simple (1 DOF) robotic hardware like motors, valves, and similar. 
  An actuator implementation is related to only one joint.
  This component type has reading and writing capabilities. Reading is not mandatory if not possible (e.g., DC motor control with Arduino board).
  The actuator type can also be used with a multi-DOF robot if its hardware enables modular design, e.g., CAN-communication with each motor independently.


A detailed explanation of hardware components is given in the `Hardware Access through Controllers design document`_.

Hardware Description in URDF
----------------------------
The ros2_control framework uses the ``<ros2_control>``-tag in the robot's URDF file to describe its components, i.e., the hardware setup.
The chosen structure enables tracking together multiple `xacro`-macros into one without any changes. 
The example hereunder shows a position-controlled robot with 2-DOF (RRBot), an external 1-DOF force-torque sensor, and an externally controlled 1-DOF parallel gripper as its end-effector.
For more examples and detailed explanations, check `ros-controls/ros2_control_demos`_ repository and `ROS2 Control Components URDF Examples design document`_.

.. code:: xml

<ros2_control name="RRBotSystemPositionOnly" type="system">
 <hardware>
   <plugin>ros2_control_demo_hardware/RRBotSystemPositionOnlyHardware</plugin>
   <param name="example_param_write_for_sec">2</param>
   <param name="example_param_read_for_sec">2</param>
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
</ros2_control>
<ros2_control name="RRBotForceTorqueSensor1D" type="sensor">
 <hardware>
   <plugin>ros2_control_demo_hardware/ForceTorqueSensor1DHardware</plugin>
   <param name="example_param_read_for_sec">0.43</param>
 </hardware>
 <sensor name="tcp_fts_sensor">
   <state_interface name="force"/>
   <param name="frame_id">rrbot_tcp</param>
   <param name="min_force">-100</param>
   <param name="max_force">100</param>
 </sensor>
</ros2_control>
<ros2_control name="RRBotGripper" type="actuator">
 <hardware>
   <plugin>ros2_control_demo_hardware/PositionActuatorHardware</plugin>
   <param name="example_param_write_for_sec">1.23</param>
   <param name="example_param_read_for_sec">3</param>
 </hardware>
 <joint name="gripper_joint ">
   <command_interface name="position">
     <param name="min">0</param>
     <param name="max">50</param>
   </command_interface>
   <state_interface name="position"/>
   <state_interface name="velocity"/>
 </joint>
</ros2_control>


Running the Framework for Your Robot
------------------------------------
To run the ros2_control framework, do the following.
The example files can be found in the `ros2_control_demos`_ repository.

#. Create a YAML  file with the configuration of the controller manager and controllers. (`Example for RRBot <https://github.com/ros-controls/ros2_control_demos/blob/master/ros2_control_demo_robot/controllers/rrbot_forward_controller_position.yaml>`_)
#. Extend the robot's URDF description with needed ``<ros2_control>`` tags.
   It is recommended to use macro files instead of pure URDF. (`Example for RRBot <https://github.com/ros-controls/ros2_control_demos/blob/master/ros2_control_demo_robot/description/rrbot_system_position_only.urdf.xacro>`_)
#. Create a launch file to start the node with `Controller Manager`_.
   You can use a default `ros2_control node`_ (recommended) or integrate the controller manager in your software stack.
   (`Example launch file for RRBot <https://github.com/ros-controls/ros2_control_demos/blob/master/ros2_control_demo_robot/launch/rrbot_system_position_only.launch.py>`_)
   
Repositories
============
The ros2_control framework consist of the following repositories:

ros2_control
  The `ros2_control`_ repository implements the main interfaces and components of the framework mentioned in the previous sections.
  
ros2_controllers
  The `ros2_controllers`_ repository implements widely used controllers, e.g., forward controller, joint trajectory controller, differential drive controller, etc.
  
ros2_control_demos
  The `ros2_control_demos`_ repository provides examples of using the framework and templates for a smooth start with it.

Differences to ros_control (ROS1)
=================================

Hardware Structures - classes
-----------------------------

The ros_control framework uses the ``RobotHW`` class as a rigid structure to handle any hardware.
This makes it impossible to extend the existing robot with additional hardware, like sensors, actuators, and tools, without coding.

The ros2_control framework defines three different types of hardware ``Actuator``, ``Sensor`` and ``System``.
Using a combination (composition) of those basic components, any physical robotic cell (robot and its surrounding) can be described.
This also means that multi-robot, robot-sensor, robot-gripper combinations are supported out of the box.
Section `Hardware Components <#hardware-components>`__ describes this in detail.

Hardware Interfaces
-------------------

The ros_control allows only three types of interfaces (joints), i.e., ``position``, ``velocity``, and ``effort``. The ``RobotHW`` class makes it very hard to use any other data to control the robot.

The ros2_control does not mandate a fixed set of interface types, but they are defined as strings in `hardware's description <#hardware-description-in-urdf>`__.
To ensure compatibility of standard controllers, standard interfaces are defined as constants in `hardware_interface package <https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/types/hardware_interface_type_values.hpp>`__.

Controller's Access to Hardware
-------------------------------

In ros_control, the controllers had direct access to the ``RobotHW`` class requesting access to its interfaces (joints).
The hardware itself then took care of registered interfaces and resource conflicts.

In ros2_control, ``ResourceManager`` takes care of the state of available interfaces in the framework and enables controllers to access the hardware.
Also, the controllers do not have direct access to hardware anymore, but they register their interfaces to the `ControllerManager`.

Migration Guide to ros2_control
===============================

RobotHardware to Components
---------------------------
#. The implementation of ``RobotHW`` is not used anymore.
   This should be migrated to SystemInterface`_ class, or to have more granularity, `SensorInterface`_ and `ActuatorInterface`_.
   See above description of "Hardware Components" to chose the suitable strategy.
#. Decide which component type is suitable for your case. Maybe it makes sense to separate ``RobotHW`` into multiple components.
#. Implement `ActuatorInterface`_, `SensorInterface`_ or `SystemInterface`_ classes as follows:
   
   #. In the constructor, initialize all variables needed for communication with your hardware or define the default one.
   #. In the configure function, read all the parameters your hardware needs from the parsed URDF snippet (i.e., from the `HardwareInfo`_ structure). Here you can cross-check if all joints and interfaces in URDF have allowed values or a combination of values.
   #. Define interfaces to and from your hardware using ``export_*_interfaces`` functions. 
      The names are ``<joint>/<interface>`` (e.g., ``joint_a2/position``).
      This can be extracted from the `HardwareInfo`_ structure or be hard-coded if sensible.
   #. Implement ``start`` and ``stop`` methods for your hardware.
      This usually includes changing the hardware state to receive commands or set it into a safe state before interrupting the command stream. 
      It can also include starting and stopping communication.
   #. Implement `read` and `write` methods to exchange commands with the hardware.
      This method is equivalent to those from `ŔobotHW`-class in ROS1.
   #. Do not forget the ``PLUGINLIB_EXPORT_CLASS`` macro at the end of the .cpp file.
#. Create .xml library description for the pluginlib, for example see `RRBotSystemPositionOnlyHardware <https://github.com/ros-controls/ros2_control_demos/blob/master/ros2_control_demo_hardware/ros2_control_demo_hardware.xml>`_.


Controller Migration
--------------------
An excellent example of a migrated controller is the `JointTrajectoryController`_.
The real-time critical methods are marked as such.

#. Implement `ControllerInterface`_ class as follows:
   #. If there are any member variables, initialized those in the constructor.
   #. In the `init` method, first call ``ControllerInterface::init`` initialize lifecycle of the controller.
      Then declare all parameters defining their default values.
   #. Implement the ``state_interface_configuration()`` and ``command_interface_configuration()`` methods.
   #. Implement ``update()`` function for the controller. (**real-time**)
   #. Then implement required lifecycle methods (others are optional):
      * ``on_configure`` - reads parameters and configures controller.
      * ``on_activate`` - called when controller is activated (started) (**real-time**)
      * ``on_deactivate`` - called when controller is deactivated (stopped) (**real-time**)
   #. Do not forget ``PLUGINLIB_EXPORT_CLASS`` macro at the end of the .cpp file.
#. Create .xml library description for the pluginlib, for example see `JointTrajectoryController <https://github.com/ros-controls/ros2_controllers/blob/master/joint_trajectory_controller/joint_trajectory_plugin.xml>`_.



.. _ros-controls/ros2_control: https://github.com/ros-controls/ros2_control
.. _ros-controls/ros2_controllers: https://github.com/ros-controls/ros2_controllers
.. _ros-controls/ros2_control_demos: https://github.com/ros-controls/ros2_control_demos
.. _controller_manager_msgs: https://github.com/ros-controls/ros2_control/tree/master/controller_manager_msgs
.. _Controller Manager: https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/controller_manager.cpp
.. _ControllerInterface: https://github.com/ros-controls/ros2_control/blob/master/controller_interface/include/controller_interface/controller_interface.hpp
.. _ros2_control node: https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/ros2_control_node.cpp
.. _ForwardCommandController implementation: https://github.com/ros-controls/ros2_controllers/blob/master/forward_command_controller/src/forward_command_controller.cpp
.. _Resource Manager: https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/src/resource_manager.cpp
.. _LifecycleNode-Class: https://github.com/ros2/rclcpp/blob/master/rclcpp_lifecycle/include/rclcpp_lifecycle/lifecycle_node.hpp
.. _JointTrajectoryController: https://github.com/ros-controls/ros2_controllers/blob/master/joint_trajectory_controller/src/joint_trajectory_controller.cpp
.. _Node Lifecycle Design: https://design.ros2.org/articles/node_lifecycle.html
.. _ros2controlcli: https://github.com/ros-controls/ros2_control/tree/master/ros2controlcli
.. _Hardware Access through Controllers design document: https://github.com/ros-controls/roadmap/blob/master/design_drafts/hardware_access.md
.. _ROS2 Control Components URDF Examples design document: https://github.com/ros-controls/roadmap/blob/master/design_drafts/components_architecture_and_urdf_examples.md

.. _ActuatorInterface: https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/actuator_interface.hpp
.. _SensorInterface: https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/sensor_interface.hpp
.. _SystemInterface: https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/system_interface.hpp
.. _HardwareInfo: https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/hardware_info.hpp


.. |ros2_control_architecture| image:: images/components_architecture.png
   :alt: "ros2_control Architecture"

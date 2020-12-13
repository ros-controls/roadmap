===========================
ros2_control Framework
===========================

The ros2_control is a framework for (real-time) control of robot using `ROS` (`Robot Operating System <http://ros.org>`__).
Its packages are a rewrite of `ros_control <http://wiki.ros.org/ros_control>`__ packages to simplify integration of a new hardware and overcome some drawbacks.

.. contents:: Table of Contents
   :depth: 2
   
Overview
========

The following figure shows the Archtecture of ros2_control framework.

![ros2_control Achitecture][ros2_control_framework_architecture]

[ros2_control_arch_resource_manager]: images/components_architecture.png "ros2_control Architecture"

Controller Manager
------------------


Resource Manager
----------------


Controllers
-----------


Hardware Components
===================


Hardware Description in URDF
----------------------------



Differences to ros_control (ROS1)
=================================

Hardware Structures - classes
-----------------------------

The ros_control usese ``RobotHW`` class as rigid structure to handle any hardware.
This makes impossible to extend exiting robot with additional with additional hardware, like sensors, actuators, and tools, without coding.

The ros2_control defines three different types of hardware ``Actuator``, ``Sensor`` and ``System``.
Using a combination (composition) of those basic components any physical robotic cell (robot and its surrounding) can be described.
This also means that multi-robot, robot-sensor, robot-gripper combinations are supported out of the box.
Section `Hardware Components <#hardware-components>`__ describe this in detail.

Hardware Interfaces
-------------------

The ros_control allows only three types of interfaces (joints), i.e., ``position``, ``velocity``, and ``effort``, the ``RobotHW`` class makes it very hard to use any other data to control the robot.

The ros2_control does not mandate fixed set of interface types, but they are defined as strings in `hardware's description <#hardware-description-in-urdf>`__.
To ensure compatibility of standard controllers, standard interfaces are defined as constants in `hardware_interface package <https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/types/hardware_interface_type_values.hpp>`__.

Controller's Access to Hardware
-------------------------------

In ros_control the controllers had direct access to the ``RobotHW`` class requesting access to its interfaces (joints).
The hardware itself than took care about registered interfaces and resource conflicts.

In ros2_control ``ResourceManager`` takes care about state of available interfaces in the framework and enable controller to access the hardware.
Also, the controllers does not have direct access to hardware anymore, but they register their interfaces to the `ControllerManager`.

Migration Guide
---------------

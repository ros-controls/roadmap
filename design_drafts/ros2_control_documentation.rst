===========================
ros2_control Framework
===========================

The ros2_control is a framework for (real-time) control of robot using `ROS` (`Robot Operating System <http://ros.org>`__).
Its packages are a rewrite of `ros_control <http://wiki.ros.org/ros_control>`__ packages to simplify integration of a new hardware and overcome some drawbacks.

If you are not familiar with the control theory, please get some idea about it (e.g. at `Wikipedia <https://en.wikipedia.org/wiki/Control_theory>`) to get familiar with the terms used in this manual.

.. contents:: Table of Contents
   :depth: 2
   
Overview
========

The ros2_control framework's source can be found in `ros-controlls/ros2_control`_ and `ros-controls/ros2_controllers`_ GitHub-repositories.
The following figure shows the Archtecture of ros2_control framework.

|ros2_control_architecture|

Controller Manager
------------------
The `Controller Manager`_ (CM) connects the controllers' and hardware-abstraction sides of the ros2_control framework.
It also serves as the entry-point for users through ROS services.
The CM implements a node without an executor so it can be integrated in custom setup.
Still, for standard user it is recommended to use default node-setup implemented in `ros2_control_node <https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/ros2_control_node.cpp>`_ file from the ``controller_manager`` package.
This manual assumes that you use this default node-setup.

On the one side, CM manages (e.g., loading, activating, deactivating, unloading) controllers and from them required interfaces.
On the other side, it has access to the hardware components (through Resource Manager), i.e., the interfaces provided by them.
The Controller Manager matches those two sides, enables controller to access the hardware's interfaces when activated or reports an error if there is a access conflict.


Resource Manager
----------------
The 


.. _overview-controllers:
Controllers
-----------
The controlles are objects derived from `ControllerInterface`_ (``controller_interface`` package in `ros-controls/ros2_control`_) and exported as plugins using ``pluginlib``-library.
For example of on controller check `ForwardCommandController implementation`_ in the `ros-controls/ros2_controllers`_ repository.

User Interfaces
---------------


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



.. _ros-controls/ros2_control: https://github.com/ros-controls/ros2_control
.. _ros-controls/ros2_controllers: https://github.com/ros-controls/ros2_controllers
.. _ros-controls/ros2_control_demos: https://github.com/ros-controls/ros2_control_demos
.. _Controller Manager: https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/controller_manager.cpp
.. _ControllerInterface: https://github.com/ros-controls/ros2_control/blob/master/controller_interface/include/controller_interface/controller_interface.hpp
.. _ForwardCommandController implementation: https://github.com/ros-controls/ros2_controllers/blob/master/forward_command_controller/src/forward_command_controller.cpp
.. _Resource Manager: 

.. |ros2_control_architecture| image:: images/components_architecture.png
   :alt: "ros2_control Architecture"

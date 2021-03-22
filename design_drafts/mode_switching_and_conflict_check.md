# Mode switching and conflict checking

## Control mode for a joint

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

## Software properties

### General properties 
A control mode is a control scheme applied to an actuator. It can use one or several state and control interfaces. 
It is *NOT* possible to mix control mode together for one actuator. 

To switch from one control mode to another control mode on one actuator, it is necessary to check if this is feasible from the viewpoint of the actuator itself. 

Often it is dangerous to switch control mode for several actuators at once (due to self-collision for instance).
The robot has to be in a specific state. This is usually done at the system level (controllers + hardware) that such safety is enforced. 
This can not be done at the control mode switching.

### Mode switching for an actuator

To be accepted the mode switching for a node can go through a method called:
```
return_type
RRBotSystemQuadrupedHardware::accept_command_resource_claim
(const std::vector<std::string> & interfaces)
```
where the set of control interfaces needed for the mode is provided in the interface.
This follows the proposal of @mahaarbo https://github.com/ros-controls/ros2_control/pull/322

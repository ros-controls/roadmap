# Flexible Joint State Messages

## Motivation

Throughout `ros_control`, only the three common fields are used for each joint and actuator.
These are position, velocity and effort.
This is also reflected in [`sensor_msgs/JointState`](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html), cementing this setup in place.
This design is proposing to break with this practice and provide a message that can accommodate reported readings for an arbitrary set of joints or hardware interfaces.
The proposed setup should both allow for reporting values for user-defined interfaces as well as making reporting position, velocity and effort optional.
The rationale behind the latter is that it is currently left entirely to user-policy whether e.g. velocity and effort should be filled with zeroes when only position is reported and how to tell if this is the case.
The flexible joint state message shall encompass a complex  hardware setup, covering a mix of different actuators and sensors.
A set of mixed-type joints would e.g. be a mobile manipulator.
One could think of a diff drive controller only providing velocity values, whereas the manipulator provides the full joint state (`position`, `velocity`, `effort`) for its joints.
In the example given, it would be possible to provide pre-defined fields in a statically defined joint state message, however this brings two problems with it:
1. How can the user indicate that a certain field (e.g. `effort`) is not provided for the joint?
Setting the value to a specific defined value such as `NaN` brings problems, latest when trying to serialize that message.
2. What if neither of the provided fields are reported by the hardware?
One example could be a suction gripper, which may report only the applied pressure or vacuum level, in which case it's again up to the user's creativity to violate one of the existing fields.

## Goal

With the given motivation, we therefore propose to introduce a flexible way of reporting values in the joint state message to address the given problems mentioned above.

Further in this text `value-identifier` is used to refer to a type of value reported in joint states, e.g. `position`, `velocity` and `effort`.

The requirements for a flexible joint state message are fulfilled, if it
1. does not enforce defining position, velocity and effort for every joint,
2. supports different hardware interface(s) additionally to position, velocity and effort,
3. does not enforce defining values for every joint in every value-identifier,
4. provides a realtime-friendly way of use (some assumptions are fair to use),

If the requirements are all fulfilled, the flexible joint state message can replace the currently used `JointStateHandles`.
One can consider the flexible joint state message to represent not only a ROS message, which being used for communication.
The same message type can be used internally in `ros_control` as a storage instance which is used to propagate the robot's state to the loaded controllers and to convey desired joint/actuator commands to the robot hardware.
That is, the hardware interface can fill the joint state message with all information the underlying hardware reports.
This would traditionally be joint states such as `position` and `velocity`, but also provides the flexibility to be extended by non-classical hardware interfaces such as grippers or custom sensors/actuators.
The joint state message would then be passed to each individual controller to read the appropriate information about of the message and perform their calculation.
The result of each controller can similarly be stored in the same instance of the joint state message which will be eventually passed back to the hardware interface.
The hardware interface then can extract the result of the controller's computation from the message and apply their values to the hardware.
That not only allows to easily combine multiple controllers, but also a dynamic composition of the hardware interfaces to contain multiple interfaces to various hardware components such as robots with interchangeable grippers.
This composition can become extremely handy when dealing with external components to a robot, such as a gripper or the previously mentioned mobile manipulator, where the base platform is not necessarily coupled with the robotic arm.

For more details on how the flexible joint state message can be used to create complex controller schemes, please refer to [controller execution management](controller_execution_management.md).

## Design

The main advantage of `sensor_msgs/JointState` is that it provides a fairly flat structure which, given some assumptions are met, it is easy to process in realtime-safe code.
Obviously, with the associate array it is easy to look up joint values for a name as its indices are aligned, i.e. the values in `position`, `velocity` and `effort` are aligned with the joint names in `name`.

```
Header header

string[] name
float64[] position
float64[] velocity
float64[] effort
```

The following proposal reflects a map-like (similar to what `std::unordered_map` represents in C++ or a hash table in other languages) datatype where not only three statically chosen value-identifiers are attached to a joint name, but can be set dynamically.

### Proposal for a Flexible Joint State Message

```
Header header

string[] interface_name
InterfaceValue[] interface_value
  string[] value_identifier
  float64[] value
```

where `interface_name` stands for a list of joint or hardware interface names, `interface_value` is a vector of a new message, being essentially a key-value pair, e.g. `effort` and `1.0`.

In the example of the mobile manipulator, a possible configuration would look like the following:

```
Header header

interface_name = ['wheel_left', 'wheel_right', 'joint_1', 'joint_2', …, 'joint_N', 'gripper_1']
interface_value = [wheel_left_iv, wheel_right_iv, joint_1_iv, joint_2_iv, …, joint_n_iv, gripper_1_iv]
```
where the `InterfaceValue` messages can be composed individually per `interface_name`.
The name is similar to the already existing joint state message coming from the URDF.

The `InterfaceValue` for the left wheel `wheel_left_iv` could be composed as such, reporting only its velocity:
```
value_identifier = ['velocity']
value = [1.5]
```

The `InterfaceValue` for the first joint `joint_1_iv` reports a more complete interface:
```
value_identifier = ['position', 'velocity', 'effort']
value = [1.1, 2.2, 3.3]
```

Additionally, the `InterfaceValue` for the gripper can be completely custom:
```
value_identifier = ['vacuum_level', 'voltage']
value = [4.4, 5.5]
```

That allows a relatively straight forward lookup of dynamically attached values per interface name.
Each individual field is thus indexable by a tuple of `interface_name` and `value_identifier`.

## Issues Considered

### Name Lookup

*Value Identifier Convention*
With a dynamically allocated message values, the value identifier might not comply to any standard.
Imagine a torque controller working with a particular hardware setup (and thus a particular joint state message configuration).
This controller needs to know whether it can compute a torque value based on `position`, `velocity` or something completely different.
An additional drawback of this key-value pair is that the keys are not necessarily defined correctly.
It can potentially be very cumbersome to debug the difference between a keys like `position`, or `pos`, or `pos_val`.

*Constants within the Message*
A potential solution to this is to provide constants of the most applicable keys and use as such.
Secondly, in order to pre-process whether all key-values pairs are available before, we propose a change to the controller interface to provide the used keys during startup time.
This would allow to perform a sanity check before actually running the controller and potentially crash.
For more details on this, we refer to the [Execution Management Design Doc](controller_execution_management.md).
Additionally, we could provide a set of helper functions in the form of header-only files along with `control_msgs`.
Nothing says that a messages package cannot ship some util code, too.

### Realtime Access Control

*Concurrent Access*
With multiple controllers being loaded, this could potentially lead to concurrent access of the same value in the joint state map.
In order to avoid this, the current ROS1 features `JointInterfaceHandles` which essentially look the resource to avoid multiple access to the same hardware interface.
However, as previously motivated this has drawbacks when designing complex control schemes, such as controller chaining or cascading.
Instead of `mutex`ing and therefore locking the resources, we can provide a lock-free method.
The idea is to introduce so-called controller groups, which either run controllers in parallel or sequentially.
In the first case, a copy of the joint state map is attached to each controller and eventually their results will be combined into a new joint map.
In the second case, the sequential controller group is responsible to attach the joint state map to the controllers in order - this can be done without any additional copies.
For more information on this, the reader is again referred to [Execution Management](controller_execution_management.md).

*Read-Only vs Read-Write Access*
In certain cases, one might want to flag certain value identifiers as read-only, such as `joint_1_position` or `joint_1_velocity` as these values are directly measured by the robot's hardware and should not be modified during the execution of the control loop.
Controllers trying to write a read-only value are thus denied and the integrity of the read-only values can be guaranteed.
This could potentially be achieved by requiring that each controller has to declare its identifiers as `input` and `output` where all `input` values are by default read-only.
Similarly, `output` identifiers are `read-write`, which would also cope with cases where an `input` topic is getting modified, e.g. when clamping joint torque commands, where input and output are the same identifier.

However, the method mentioned here is a sole implementation detail and is not reflected in the actual message.
For this, one could thing of creating a third field to the `InterfaceValue` message, which has a `enum` value for IO:
```
Header header

string[] interface_name
InterfaceValue[] interface_value
  int8 IO_READ_ONLY = 0  # read-only constant
  int8 IO_READ_WRITE = 1 # read-write constant
  string[] value_identifier
  float64[] value
  int8[] io_flag
```

## Discarded Designs

### Flat Structure
```
Header header

string[] value_identifier
float64[] value
```

Alternatively to the chosen one, this proposal has one less indirection when indexing the values.
Each value has to be precisely identified by its name.

While being small and flat, this proposal was discarded as the indexing entails more issues than benefits as for every value per hardware interface, a unique name has to be designed.
This design would most likely mean to prefix the value with its interface name, e.g. `joint_1_torque_command`, which is no better than having a level of indirection more in the index.
Furthermore, this even more leads to confusion and non-standard value identifiers.

### Matrix configuration
```
Header header

string[] joint_name
string[] interface_name
Matrix2D value
```

where ideally `Matrix2D` is a double indexable type, where the rows/columns are defined by `joint_name` and the columns/rows are defined by `interface_name`.

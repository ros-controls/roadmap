# Flexible Joint State Messages

## Motivation

Throughout `ros_control`, only the three common fields are used for each joint and actuator.
These are position, velocity and effort.
This is also reflected in [`sensor_msgs/JointState`](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html), cementing this setup in place.
This design is proposing to break with this practice and provide a message that can accommodate reported readings for an arbitrary set of joints or hardware interfaces.
The proposed setup should both allow for reporting values for user-defined interfaces as well as making reporting position, velocity and effort optional.
The rationale behind the latter is that it is currently left entirely to user-policy whether e.g. velocity and effort should be filled with zeroes when only position is reported and how to tell if this is the case.
The flexible joint state message shall encompass a complex  hardware setup, covering a mix of different actuators and sensors.
A set of mixed-type joints would e.g. be a mobile actuator.
One could think of a diff drive controller only providing velocity values, whereas the manipulator provides the full joint state (`position`, `velocity`, `effort`) for its joints.
In the example given, it would be possible to provide pre-defined fields in a statically defined joint state message, however this brings two problems with it:
1. How can the user indicate that a certain field (e.g. `effort`) is not provided for the joint?
Setting the value to a specific defined value such as `NaN` brings problems, latest when trying to serialize that message.
2. What if neither of the provided fields are reported by the hardware?
One example could be a suction gripper, which may report only the applied pressure or vacuum level, in which case it's again up to the user's creativity to violate one of the existing fields.

## Goal

With the given motivation, We therefore propose to introduce a flexible way of providing custom values in the joint state message to address the given problems mentioned above.

Further in this text `value-identifier` is used to refer to a type of value reported in joint states, e.g. `position`, `velocity` and `effort`.

To define a new message that
1. does not enforce defining position, velocity and effort for every joint,
1. supports different hardware interface to report additional values on top of position, velocity and effort,
3. does not enforce defining values for every joint in every value-identifier,
4. provides a realtime-friendly way of use (some assumptions are fair to use),

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

The following proposal reflects a map-like (similar to what `std::unordered_map` represents in C++ or a hash table in other languages) datatype where not only three statically chosen value-identifier are attached to a joint name, but can be set dynamically.

### Proposal for a Dynamic Joint State Message

```
Header header

string[] interface_name
InterfaceValue[] interface_value
  string[] value_identifier
  float64[] value
```

where `name` stands for a list of joint or hardware interface names, `interface_value` is a vector of a new message, being essentially a key-value pair, e.g. `effort` and `1.0`.

In the example of the mobile manipulator, a possible configuration would look like the following:

```
Header header

interface_name = ['wheel_left', 'wheel_right', 'joint_1', 'joint_2', …, 'joint_N', 'gripper_1']
InterfaceValue = [wheel_left_iv, wheel_right_iv, joint_1_iv, joint_2_iv, …, joint_n_iv, gripper_1_iv]
```
where the `InterfaceValue` messages can be composed individually per `InterfaceName`.
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
value_identifier = ['vaccuum_level', 'voltage']
value = [4.4, 5.5]
```

That allows a relatively straight forward lookup of dynamically attached values per interface name.
Each individual field is thus indexable by a tuple of `interface_name` and `value_identifier`.

## Issues Considered

### Name Lookup

With a dynamically allocated message values, the value identifier might not comply to any standard.
Imagine a torque controller working with a particular hardware setup (and thus a particular joint state message configuration).
This controller needs to know whether it can compute a torque value based on `position`, `velocity` or something completely different.
An additional drawback of this key-value pair is that the keys are not necessarily defined correctly.
It can potentially be very cumbersome to debug the difference between a keys like `position`, or `pos`, or `pos_val`.

*possible improvements:*
A potential solution to this is to provide constants of the most applicable keys and use as such.
Secondly, in order to pre-process whether all key-values pairs are available before, we propose a change to the controller interface to provide the used keys during startup time.
This would allow to perform a sanity check before actually running the controller and potentially crash.
For more details on this, we refer to the [Execution Management Desin Doc](controller_execution_management.md).
Additionally, we could provide a set of helper functions in the form of header-only files along with `control_msgs`.
Nothing says that a messages package cannot ship some util code, too.

### Read Only Access

// TODO, explain that with controller chaining, the resource management has to be addressed that values should be declared read-only

## Discarded Designs

### Matrix configuration
```
Header header

string[] joint_name
string[] interface_name
Matrix2D value
```

where ideally `Matrix2D` is a double indexable type, where the rows/columns are defined by `joint_name` and the columns/rows are defined by `interface_name`.

# Flexible Joint State Messages

## Motivation

Throughout `ros_control`, only the three common fields are used for each joint and actuator.
These are position, velocity and effort.
This is also reflected in [`sensor_msgs/JointState`](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html), cementing this setup in place.
This design is proposing to break with this practice and provide a message that can accommodate reported readings for an arbitrary set of joints and joint interfaces.
The proposed setup should both allow for reporting values for user-defined interfaces as well as making reporting position, velocity and effort optional.
The rationale behind the latter is that it is currently left entirely to user-policy whether e.g. velocity and effort should be filled with zeroes when only position is reported and how to tell if this is the case.
A set of mixed-type joints would e.g. have zeroes or NaNs for one joint in the position vector and effort but only fill velocity with proper values.

Further in this text `value-type` is used to refer to a type of value reported in joint states, e.g. position, velocity and effort.

Assumptions in the current setup:
* the values in `position`, `velocity` and `effort` are aligned with the joint names in `name`.

## Goal

To define a new message that
1. allows to report additional values on top of position, velocity and effort,
2. does not enforce defining position, velocity and effort for every joint,
3. does not enforce defining values for every joint in every value-type,
4. provides a realtime-friendly way of use (some assumptions are fair to use),

## Design

The main advantage of `sensor_msgs/JointState` is that it provides a fairly flat structure which, given some assumptions are met, it is easy to process in realtime-safe code.

```
Header header

string[] name
float64[] position
float64[] velocity
float64[] effort
```

### First proposal

```
Header header

string[] name
InterfaceValue[] interface_value
  string interface_name
  float64[] value
```

where `name` stands for a list of joint names, `interface_value` is a vector of a new message, holding the name of the interface and a list of values.
Assumptions:
* `interface_name` needs to be kept standard somehow
* the values in `value` are aligned with the joint names in `name`.

Possible improvements:
* Turn `interface_name` into an enum and define it in a message.
This would be problematic as we are - yet again - baking names into the system.
Extending the list would result in MD5 changes in this message too.
* Support a set of helper functions in the form of header-only files along with `control_msgs`.
Nothing says that a messages package cannot ship some util code, too.

### Second proposal

```
Header header

string[] joint_name
string[] interface_name
Matrix2D value
```

where ideally `Matrix2D` is a double indexable type, where the rows/columns are defined by `joint_name` and the columns/rows are defined by `interface_name`.

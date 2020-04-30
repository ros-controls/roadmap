# Joint/actuator interfaces and transmission parsing

In ROS Melodic and previous distributions, `ros_control` operates with statically-typed joint handles as the primary mean of communication within components of the framework. 
These handles are stored in joint-name to handle maps and are made available to controllers inheriting (or using other means to acquire) the relevant controller interface class e.g.
```
  class DiffDriveController : 
    public controller_interface::Controller<hardware_interface::VelocityJointInterface>
  { ... };
```

Commands on joint interfaces are transformed into commands for actuator interfaces using transmissions.
An example of such a transmission is simple reducer which applies a multiplier between commands and readings.
Users of `ros_control` can extend the framework by providing their own transmissions as all transmissions are loaded as plugins.
When defining a robot setup, joints and actuators are connected via transmissions declared through the URDF along with the required interfaces.
At startup, the required interfaces for transmissions are validated by the `TransmissionLoader`.
Transmissions use a [common interface](https://github.com/ros-controls/ros_control/blob/melodic-devel/transmission_interface/include/transmission_interface/transmission.h) to propagate values.
This however already shows a shortcomings of this design as a whole as with a fixed set of interfaces, a fixed set of functions were created, most of them defined empty, for example:
```
  virtual void jointToActuatorEffort(const JointData&    jnt_data,
                                           ActuatorData& act_data) = 0;
```
All transmissions inheriting this interface has to define empty functions for those it does not operate on. Additionally, this set of functions is not extendable in a clean way, without incurring additional interface inheritance ([and a general mess...](https://github.com/ros-controls/ros_control/pull/395)) on the derived classes.
 
## Scope:
This design document proposes changes to 
1) joint and actuator interfaces
2) transmission parsing
3) controller interfaces (as a consequence of 1)

and ties in with the use of the [flexible joint states message](https://github.com/ros-controls/roadmap/blob/master/design_drafts/flexible_joint_states_msg.md) 

4) in the context of replacing `ActuatorData` and `JointData` from `ros_control` on ROS Melodic in how transmission loading and runtime is handled
5) the `joint_state_controller` which depends on the "read-only" `JointStateHandle` class to report position, velocity and effort values.

## Proposal

### urdf

```
<transmission name="extended_simple_trans">
    <type>transmission_interface/ExtendedSimpleTransmission</type>
    <joint name="joint1">
      <actuator>
        <!-- control input (u) for the joint, i.e. effort/torque value -->
        <hardwareInterface>hardware_interface/EffortJointCommandInterface</hardwareInterface>
      </actuator>
      <sensor>
        <!-- output state (x) from the joint, e.g. position -->
        <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
        <!-- output state (x_dot) from the joint, e.g. velocity -->
        <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
        <!-- output state (x_dot_dot) from the joint, e.g. acceleration -->
        <hardwareInterface>hardware_interface/AccelerationJointInterface</hardwareInterface>
      </sensor>
    </joint>
    <joint name="joint2">
      <actuator>
        <hardwareInterface>hardware_interface/EffortJointCommandInterface</hardwareInterface>
      </actuator>
      <sensor>
        <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
        <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
        <hardwareInterface>hardware_interface/AccelerationJointInterface</hardwareInterface>
      </actuator>
    </joint>
    <joint>
      <actuator>
        <!-- control input (u) for the joint, i.e. velocity value -->
        <hardwareInterface>hardware_interface/VelocityJointCommandInterface</hardwareInterface>
      </actuator>
      <sensor>
        <!-- custom output state (x) for the joint, i.e. some custom value such as current      
        <hardwareInterface name="current_joint_interface">hardware_interface/CustomValueInterface</hardwareInterface>
      </sensor>
    </joint>
</transmission>
```

The content of a `hardwareInterface` XML tag should match up with strings provided by `ros2_control` in the shape of familiar names:
`hardware_interface::PositionJointInterface, hardware_interface::VelocityJointInterface, hardware_interface::EffortJointInterface`.
By relying on strings internally in the framework but using them mostly through shared names, a standard nomenclature is kept all the while allowing custom extensions in an easy and transparent way. 

### RobotHardware

Based on the URDF snippet above, the instance of a robot hardware can be generated as described in the [hardware interface design doc](hardware_interface.md).

The transmission parser is being used to set up and configure the robot hardware accordingly.
Refer to the design doc for more information.

### Transmissions

The Transmission interface and base class should be merged and simplified.
There is little benefit in providing multitudes of classes, requiring multitudes of checks all following the same pattern.
I am proposing a much simpler setup as outlined below.

```
template <class JointData, class ActuatorData>
class Transmission
{
public:
  virtual ~Transmission() {}

  virtual void actuatorToJoint(const std::string& interface_name,
                               const ActuatorData& act_data,
                                     JointData&    jnt_data) = 0;

  virtual void jointToActuator(const std::string& interface_name,
                               const ActuatorData& act_data,
                                     JointData&    jnt_data) = 0;
};


template <class JointData, class ActuatorData>
class ActuatorToJointTransmissionHandle
{
public:
  ActuatorToJointTransmissionHandle(const std::string&  name,
                                    Transmission<JointData, ActuatorData>* transmission,
                                    const ActuatorData& actuator_data,
                                    const JointData&    joint_data)
    : TransmissionHandle(name, transmission, actuator_data, joint_data) {}

  void propagate()
  {
    transmission_->actuatorToJoint("hardware_interface::PositionJointInterface", actuator_data_, joint_data_);
    transmission_->actuatorToJoint("hardware_interface::VelocityJointInterface", actuator_data_, joint_data_);
    transmission_->actuatorToJoint("hardware_interface::EffortJointInterface", actuator_data_, joint_data_);

    if(my_custom_check)
    {
       transmission_->actuatorToJoint("awesome_interface/FooJointInterface", actuator_data_, joint_data_);
    }

  }
};
```

The template parameters in this case may be uncalled for in a first implementation but could serve as a base for supporting other underlying data representations, not only a single one (whether it's flexible joint states or not).

Transmission parsing should involve a much lighter weight setup as before.
The initial implementation should be a simple "registry" of required interfaces which is fairly close to how it is done at the moment.
We should aim to remove as many intermediate handlers, loaders and interfaces as possible as their added value is often minimal at a cost of large code complexity.


## Related possible changes

The proposed changes open a couple of possibilities in the framework.

Registered interfaces can also be interpreted in the scope of controller groups or controllers themselves, providing virtual joints, implementing data passing between chained controllers while offering some accounting.

Actuator and joint interfaces need not be separate things. 
On top of this, transmissions could also be thought of as controllers, only much simpler.
Perhaps not at the top level, but the underlying implementation of transmissions could leverage that of controllers, reducing the amount of code to be maintained. 
(Of course provided that it doesn't come with a high code complexity.)

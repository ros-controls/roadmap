# Hardware Access through Controllers

The following article describe how controllers can exclusively claim hardware resources yet stay as flexible as possible to avoid a strongly typed triple of 'position', 'velocity' and 'effort [c.f. [Flexible Joint States Message](https://github.com/ros-controls/roadmap/blob/master/design_drafts/flexible_joint_states_msg.md)].
We firstly describe how hardware resources are classified and loaded.
Secondly, we provide a real-time safe way of accessing these resources through controllers.

## Hardware Resources
A hardware resource describes a physical component which is being considered through ros2-control.
We hereby distinguish between three classes of hardware resources, namely Actuators, Sensor and System.
Every individual hardware is loaded at runtime and thus allows a flexible and dynamic composition of the to be controlled setup.
The hardware is composed uniquely through the URDF.

**Joint**
A joint is considered a logical component and is being actuated by at least one actuator (it might be generically under- or over-actuated depending on the actual hardware setup).
The joint is meant to be abstraction layer between a high-level controller instance and the underlaying hardware.
The abstraction is needed to shim over a potentially complex hardware setup in order to control a joint.
A single joint might be controlled by multiple motors with a non-trivial transmission interface, yet a controller only really thinks in joint state values, not in transmission ratios.

A joint is configured in conjunction to a hardware resource such as **Actuator**, **Sensor** or **System**.
The joint component is configured through command and state interfaces.
The command interfaces describe the value in which this joint can be controlled (e.g. effort or velocity) where as the state interfaces are considered the state feedback interfaces.
An example URDF:
```xml
...
<joint name="joint1">
  <command_interface>velocity</command_interface>
  <state_interface>position</state_interface>
  <state_interface>velocity</state_interface>
</joint>
...
```

**Actuator**
An actuator describes a single motor instance with at max 1DoF and is strictly tight to a **Joint**.
It might hereby take a single command value for its appropriate mode of operation, such as a desired joint velocity or effort - in rare cases an actuator might actually take a precise joint position value.
The implementation of this actuator might then convert the desired value into PWM or other hardware specific commands and control the hardware.
Similarly, an actuator might provide state feedback.
Depending the setup, the motor encoders might provide position, velocity, effort or current feedback.
The implementation likewise might provide these values by connecting through the hardware driver.

The URDF snippet for an actuator might look like the following:
```xml
<ros2_control name="my_simple_servo_motor" type="actuator">
<hardware>
  <class>simple_servo_motor_pkg/SimpleServoMotor</class>
  <param name="serial_port">/dev/tty0</param>
...
</hardware>
  <joint name="base_link">
  <command_interface>position</command_interface>
  <state_interface>position</state_interface>
  <param name="max_value">1.57</param>
  <param name="min_value">-1.57<param>
...
</joint>
</ros2_control>
```
The snippet above depicts a relative simple hardware setup, which a single actuator which controls one logical joint.
The joint hereby is configured to be commanded in position values, whereas the state feedback is also position.

If a joint is configured with a command or state interface the hardware is not supporting, a runtime error shall occur during startup.
Opposite to it, a joint might be configured with only the minimal required interfaces even though the hardware might support additional interfaces (such as "current" or "voltage").
Those shall simply be not instantiated and thus ignored.

**Sensor**
A sensor is a hardware component which only has state feedback.
It can be considered as a read-only hardware resource and thus does not require exclusive access management - that is it can be used by multiple controllers concurrently.
```xml
<ros2_control name="my_simple_sensor">
  <hardware type="sensor">
    <class>simple_sensor_pkg/SimpleSensor</class>
    <param name="serial_port">/dev/tty0</param>
    ...
  </hardware>
  <sensor name="my_sensor">
    <state_interface>roll</state_interface>
    <state_interface>pitch</state_interface>
    <state_interface>yaw</state_interface>
  </sensor>
</ros2_control>
```
Note that we technically have a separation between a physical hardware resource (`<hardware type="sensor">`) and a logical component (`<sensor name="my_sensor">`), both called *Sensor*.
We don't individually specify them further in this document as they don't have a significant semantic interpretation for the user.

**System**
A system is meant to be a more complex hardware setup which contains multiple joints and sensors.
This is mostly used for third-party robotic systems such as robotic arms or industrial setups, which have their own (proprietary) API.
The implementation of the system hardware resource thus serves as an interface between the hardware and the controller to provide this API.
```xml
<ros2_control name="MyComplexRobot" type="system">
  <hardware>
    <class>complex_robot_pkg/ComplexRobot</class>
    ...
    </hardware>
    <joint name="joint1">
      <command_interface>velocity</command_interface>
      <command_interface>effort</command_interface>
      <state_interface>position</state_interface>
      <state_interface>velocity</state_interface>
      <state_interface>effort</state_interface>
    </joint>
    <joint name="joint2">
      <command_interface>velocity</command_interface>
      <command_interface>effort</command_interface>
      <state_interface>position</state_interface>
      <state_interface>velocity</state_interface>
      <state_interface>effort</state_interface>
    </joint>
</ros2_control>
```

## Resource Manager
The resource manager is responsible for parsing the URDF and instantiating the respective hardware resources.
It has ownership over the lifetime of these hardware resources and their associated logical joints.
It serves as the storage backend for the controller manager which can loan resources to the controller.
The resource manager hereby administrates which controllers claimed which resources.
If a controller does no longer need access to the claimed resource, that resource is being released and offered for other controllers to be claimed.

The resource manager internally maintains a mapping of each individual hardware resources and their interfaces.
This mapping can be indexed through a simple `<logical_component>/<interface_name>` lookup.
As mentioned previously it abstracts the individual hardware resources from their logical components, such that a user (or controller in that sense) does not have to know which hardware is responsible for commanding a joint.

In the examples above, the actuator command interfaces are being mapped to `base_link/position`, the state interfaces equivalently to `base_link/position`.
Likewise for the sensor, their interfaces are being mapped to `my_sensor/roll`, `my_sensor/pitch`, `my_sensor/yaw`.

## Controller Interface
Once the system is bootstrapped and a controller is loaded, it can claim logical components and access their interfaces.

**Generic Access**
A controller has the chance to access a single interface value by directly indexing the resource manager for the respective key, such as `joint1/effort`, which allows to set effort values during the controllers execution.
```c++
void MyController::init(ResourceManager * resource_manager)
{
  InterfaceCommandHandle joint1_effort_cmd = resource_manager->claim_command_interface("joint1/effort");
  InterfaceStateHandle joint1_position_state = resource_manager->claim_state_interface("joint1/position");
}
```

**Semantic Components**
While the above example might suffice for simple setups, one can imagine that there might be quite some handles accumulated, e.g. when dealing with 6D FT/Sensors or IMUs.
We therefore propose semantic components which wrap a non-zero amount of keys and provide a more meaningful API on top of it.
```c++
void MyController::init(ResourceManager * resource_manager)
{
  FTSensor6D ft_sensor(resource_manager,
    "sensor1/fx", "sensor1/fy", "sensor1/fz",  // force values
    "sensor1/tx", "sensor1/ty", "sensor1/tz"); // torque values

  std::vector<double> torque = ft_sensor.get_torque_values();
}
```

As an outlook, one could think of semantic components to provide more insights to the hardware resources.
An example would be a camera sensor, where it wouldn't make much sense to provide a key for every pixel.
A solution would be provide keys which describe the camera sufficiently, such as a generic `data` and `size` key.
The implementation of this `Camera` class would require insights on how to interpret the `camera1/data` pointer to not treat it as an individual `double` value, but as a pointer address or similar.
```c++
Camera cam(resource_manager, "camera1/data", "camera1/size");
cv::Mat img = cam.get_image();
```

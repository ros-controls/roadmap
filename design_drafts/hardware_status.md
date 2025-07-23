# Standardized Hardware Status Reporting

## Short description

This proposal is an attempt at a standardized way for hardware components in `ros2_control` to report their status.

Right now, if you're writing a hardware interface, how you report things like health, errors, or connectivity is pretty much up to you. This usually means everyone rolls their own custom messages. While that works for a single project, it makes it really tough to build generic, reusable tools on top of `ros2_control` (and even internally!). This proposal is a first-pass attempt at defining a generic `HardwareStatus` message. The main goal is to find a good balance between a structured, predictable format that tools can rely on, and the flexibility needed to report all the weird, wonderful, and specific details of different hardware.

This is very much a draft to get the conversation started, not a final solution!

Here's a diagram I put together to visualize it:

![hardware_status](images/hardware_status.png)

Let's discuss this in slightly more detail.

## 1. The Idea: Structured vs. Unstructured

The core idea is to split status reporting into two complementary parts.

1. **Structured Status:**
- A fixed set of fields covering \~80% of common hardware needs—machine-readable, reliable, and directly consumable by controllers, watchdogs, automation tools and even for us internally.
- A compact, general-purpose block of enums and identifiers. If a device can’t fill one of these fields, it simply reports `UNKNOWN`.

2. **Unstructured Status:**
- A free-form array of key/value diagnostics for everything else—geared toward logs, dashboards, and human inspection only.
- A slower, richer stream of `diagnostic_msgs/KeyValue[]`, strictly for debugging and UI, ideally not parsed by control loops.

## 2. Example Message Topology

We separate **real-time status** (fast, small) from **detailed diagnostics** (bulkier, slower) on two topics:

| Topic                          | Msg Type                           | Rate     | Intended Use                                |
| ------------------------------ | ---------------------------------- | -------- | ------------------------------------------- |
| `/hardware_status`             | `control_msgs/HardwareStatus`      | 1–50 Hz  | Health, ops & safety logic, auto‐monitoring |
| `/hardware_diagnostics` | `control_msgs/HardwareDiagnostics` | 0.1–1 Hz | GUI dashboards, logs, debugging             |

## 3. Structured Status: `HardwareStatus`

```
# control_msgs/msg/HardwareStatus

std_msgs/Header header        # timestamp + frame_id (optional)
string           hardware_id  # unique per‐instance, e.g. "left_wheel/driver"

# ——— Health & Error ——————————————————————————————————————————————
uint8  health_status         # see HealthStatus enum
uint8[]  error_domain        # Array of device errors, because hardware can throw more than one, see ErrorDomain enum

# ——— Operational State ———————————————————————————————————————————
uint8  operational_mode      # see ModeStatus enum
uint8  power_state           # see PowerState enum
uint8  connectivity_status   # see ConnectivityStatus enum

# ——— Vendor & Version Info ————————————————————————————————————————
string manufacturer          # e.g. "Bosch"
string model                 # e.g. "Lidar-XYZ-v2"
string firmware_version      # e.g. "1.2.3"

# ——— Optional Details for Context —————————————————————————————————
# Provides specific quantitative values related to the enums above.
# e.g., for power_state, could have {key: "voltage", value: "24.1"}
# e.g., for connectivity, could have {key: "signal_strength", value: "-55dBm"}
diagnostic_msgs/KeyValue[] state_details
```

### 3.1. Enums

```
# control_msgs/msg/HardwareStatus (continued)

# High-level health
uint8 HEALTH_UNKNOWN=0
uint8 HEALTH_OK     =1
uint8 HEALTH_DEGRADED=2
uint8 HEALTH_WARNING =3
# Hardware stops publishing state when it returns ERROR/FATAL, how are these set/updated?
uint8 HEALTH_ERROR   =4
uint8 HEALTH_FATAL   =5

# Error category
uint8 ERROR_NONE    =0
uint8 ERROR_UNKNOWN =1
uint8 ERROR_HW # generic hardware fault/error
uint8 ERROR_SW # generic software fault/error
uint8 ERROR_OVER_TRAVEL # Hardware stopped motion because position is over limits

# Hardware/Software status
uint8 EMERGENCY_STOP_HW # state of the emergency stop hardware (i.e. e-stop button state)
uint8 EMERGENCY_STOP_SW # state of the emergency stop software system (over travel, pinch point)
uint8 PROTECTIVE_STOP_HW # state of the protective stop hardware (i.e. safety field state)
uint8 PROTECTIVE_STOP_SW # state of the software protective stop
# Some protective stop errors need to be acknowledged before the hardware can reactivate
# see https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/dashboard_client.html#unlock-protective-stop-std-srvs-trigger
uint8 SAFETY_STOP
# Some hardware requires calibration on startup (for example a linear rail or quadruped)
unit8 CALIBRATION_REQUIRED


# Mode of operation
uint8 MODE_UNKNOWN    =0
uint8 MODE_MANUAL     =1
uint8 MODE_AUTO       =2 # automatic mode when the driver is remote controlling the hardware
uint8 MODE_SAFE       =3 # what is the expected use case for this mode?
uint8 MODE_MAINTENANCE=4
uint8 MODE_JOG_MANUAL
uint8 MODE_ADMITTANCE
uint8 MODE_MONITORED_STOP
uint8 MODE_HOLD_TO_RUN
unit8 MODE_CARTESIAN_TWIST
unit8 MODE_CARTESIAN_POSE
uint8 MODE_TRAJECTORY_FORWARDING
uint8 MODE_TRAJECTORY_STREAMING

# Power states
uint8 POWER_UNKNOWN   =0
uint8 POWER_OFF       =1
uint8 POWER_STANDBY   =2
uint8 POWER_ON        =3
uint8 POWER_SLEEP     =4
uint8 POWER_ERROR     =5
# Battery power states see [BatteryState.msg](https://docs.ros2.org/foxy/api/sensor_msgs/msg/BatteryState.html)
uint8 POWER_LEVEL_LOW
uint8 POWER_LEVEL_CRITICAL
uint8 POWER_CHARGING
uint8 POWER_CHARGING_ERROR

# Connectivity
uint8 CONNECT_UNKNOWN =0
uint8 CONNECT_UP      =1
uint8 CONNECT_DOWN    =2
uint8 CONNECT_FAILURE =3
uint8 CONNECTION_SLOW # to tell the controlling system it is struggling to communicate at rate
```

#### 3.2. A Note on a Future Addition 

A potential limitation of the single-value enums above is that a component can only report one state per category at a time. Consider the `error_domain`: what happens if a hardware fault (`ERROR_HW`) immediately causes a communication failure (`ERROR_COMM`)? With the current design, the hardware driver must choose to (or is limited to) report only one.
That or return an array of errors and let mission control sort out the correct action to recover.

A potential solution for this in a future iteration would be to define some enums as **bitfields**. This would involve assigning values as powers of 2, allowing multiple states to be combined using a bitwise `OR` operation.

For example, the `ErrorDomain` enum could be redefined as a bitmask:
```
# Example ErrorDomain as a bitfield (why only an 8 bit number?)
uint8 ERROR_NONE    = 0  # 0b00000000
uint8 ERROR_HW      = 1  # 0b00000001
uint8 ERROR_FW      = 2  # 0b00000010
uint8 ERROR_COMM    = 4  # 0b00000100
uint8 ERROR_POWER   = 8  # 0b00001000
# ... up to 4 more flags
```

A publisher could then report both a hardware and power fault simultaneously by setting the value to `ERROR_HW | ERROR_POWER` (which is `9`, or `0b00001001`). A subscriber could then check for a specific error using a bitwise `AND` (e.g., `if (status.error_domain & ERROR_HW)`).

The primary trade-off is that we would be limited by the size of the enum's underlying type. A `uint8` allows for exactly 8 unique flags. While this may be sufficient for now, it's a constraint to keep in mind as we finalize this design. We can add this if we hear from the community that this is needed.

## 4. Unstructured Diagnostics: `HardwareDiagnostics`

```
# control_msgs/msg/HardwareDiagnostics

std_msgs/Header    header
string             hardware_id
KeyValue[]         entries   # diagnostic_msgs/KeyValue[]
```

> **Example Entry**
>
> ```yaml
> header:
>   stamp: {sec: 1625563200, nanosec: 0}
> hardware_id: "arm_controller"
> entries:
>   - {key: "cpu_temp",       value: "72.5°C"}
>   - {key: "voltage_input",  value: "24.1V"}
>   - {key: "last_error_code",value: "0x1A3F"}
> ```

## 5. Open Questions & Discussion

1. Could we reuse `lifecycle_msgs/State` for `operational_mode`, or is a dedicated enum preferable for clarity?
2. I left some question marks in the diagrams, any categories we are missing?
3. Should `HardwareStatus` include a short `string error_message`, or strictly push error details into diagnostics only?
4. Also another thing, maybe we use one big message (`control_msgs/HardwareStatus`) to make it simpler rather than publish structured vs. unstructured data on separate topics (`/hardware_status` and `/hardware_diagnostics`)?
5. And the main questions that I have, Is this whole approach overly complicated, let's avoid that pitfall.

Looking forward to hearing what everyone thinks!

## Hardware Status Interface
What does configuring the Hardware Status (per hardware because a mobile_base is likely different than the arm mounted on top of it) look like?
Should we have blocks of state (i.e. standardized messages) that can be added together if the hardware offers X, Y and Z features?
(For example my robot arm has a `safety interface` for e-stop/p-stop and a `hardware_status` interface to report power, operating mode and ...)
What does it look like at the interface level? Is there a separate read (and maybe write) method for status reporting and reconfiguration?
For example standard safety status (E-stop, P-stop), operating mode, [battery state](https://docs.ros2.org/foxy/api/sensor_msgs/msg/BatteryState.html).

JointState has been the standard ROS2 control works with. What about GPIO, SafetyStatus, BatteryState, .... these are interfaces that hardware frequently provides.
What if ros2_control made a set of messages to standardize it's interfaces for each subcategory?
[SensorMsgs](https://docs.ros2.org/foxy/api/sensor_msgs/index-msg.html) is a start of what we need.
For example the UR controller exposes lots of interfaces via [GPIO](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/85d2ad8d1526ee6c0f21dca94e1e697c83706b71/urdf/ur.ros2_control.xacro#L294-L311) but not in a standardized way so if someone wanted to control it and then switch robots their codebase would likely need to change to handle auxiliary control and monitoring.

Some errors or states will be set as the hardware stops functioning.
Should the status broadcaster hold and continue to publish last known state?
Should the status broadcaster offer statistics on hardware DEACTIVATE/ERROR and ACTIVATIONS?
Lots of industrial applications would like to know how many e-stops, number of controller errors/faults, ____ per shift, week or some period of time
Could this open the option for custom or standard controllers to monitor and keep the system healthy? i.e. automatic arm fault reset controller,

Links of hardware interfaces and their attempt to convey hardware status and support other control modes

### UR
[hardware_interface](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/868f240bc8578ebfa1d19b94f8a6a1ad62fa0bd1/ur_robot_driver/src/hardware_interface.cpp#L266-L270)
[SafetyMode.msg](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_dashboard_msgs/msg/SafetyMode.msg)
[RobotMode.msg](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_dashboard_msgs/msg/RobotMode.msg)
[control.xacro](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/blob/85d2ad8d1526ee6c0f21dca94e1e697c83706b71/urdf/ur.ros2_control.xacro#L294-L311)

### Kuka
[hardware_interface](https://github.com/lbr-stack/lbr_fri_ros2_stack/blob/f2784b86e5975eddc9b5eab901baaca329306653/lbr_ros2_control/include/lbr_ros2_control/system_interface_type_values.hpp#L8-L27)

### Kinova
[fault_reset controller](https://github.com/Kinovarobotics/ros2_kortex/blob/main/kortex_description/arms/gen3/7dof/config/ros2_controllers.yaml#L17-L18) to report and reset faults.
[twist_controller](https://github.com/Kinovarobotics/ros2_kortex/blob/309f9c9d4a277970e542e5ac1fe260ced0630f65/kortex_description/arms/gen3/7dof/config/ros2_controllers.yaml#L11-L12)

### Dynamixel
[hardware_interface](https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface/blob/02841dd2ae422676e5dc0fea37057bdec3be8cc1/include/dynamixel_hardware_interface/dynamixel_hardware_interface.hpp#L53-L91)

### Robotiq
[hardware_interface](https://github.com/PickNikRobotics/ros2_robotiq_gripper/blob/12e623212e6891a5fcc9af94d67b07e640916394/robotiq_driver/include/robotiq_driver/driver.hpp#L41-L66)
[acrivation_controller](https://github.com/PickNikRobotics/ros2_robotiq_gripper/blob/main/robotiq_controllers/src/robotiq_activation_controller.cpp)

### Ethercat
[hardware_interface](https://github.com/ICube-Robotics/ethercat_driver_ros2/blob/52be2c2ed163bab25d46c402ddb4e7216c0a0ec3/ethercat_generic_plugins/ethercat_generic_cia402_drive/include/ethercat_generic_plugins/cia402_common_defs.hpp#L31-L56)

### ROS2 canopen driver
https://github.com/ros-industrial/ros2_canopen/tree/master

### Picknik Twist & Fault controllers
https://github.com/PickNikRobotics/picknik_controllers

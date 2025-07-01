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
diagnostic_msgs/KeyValue[] health_diagnostics # more info on health

uint8  error_domain          # see ErrorDomain enum
diagnostic_msgs/KeyValue[] error_diagnostics # more info on error like HW error codes and FW error codes

# ——— Operational State ———————————————————————————————————————————
uint8  operational_mode      # see ModeStatus enum
diagnostic_msgs/KeyValue[] operational_diagnostics # The operational mode information like hybrid mode, position, position-velocity, gain scheduling being on etc

uint8  power_state           # see PowerState enum
diagnostic_msgs/KeyValue[] power_diagnostics # To log more information like the battery voltage, current, charging rate etc

uint8  connectivity_status   # see ConnectivityStatus enum
diagnostic_msgs/KeyValue[] connectivity_diagnostics # To log information like signal strength and more

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
uint8 HEALTH_ERROR   =4
uint8 HEALTH_FATAL   =5

# Error category
uint8 ERROR_NONE    =0
uint8 ERROR_UNKNOWN =1
uint8 ERROR_HW      =2
uint8 ERROR_FW      =3
uint8 ERROR_COMM    =4
uint8 ERROR_POWER   =5

# Mode of operation
uint8 MODE_UNKNOWN    =0
uint8 MODE_MANUAL     =1
uint8 MODE_AUTONOMOUS =2
uint8 MODE_SAFE       =3
uint8 MODE_MAINTENANCE=4

# Power states
uint8 POWER_UNKNOWN   =0
uint8 POWER_OFF       =1
uint8 POWER_STANDBY   =2
uint8 POWER_ON        =3
uint8 POWER_SLEEP     =4

# Connectivity
uint8 CONNECT_UNKNOWN =0
uint8 CONNECT_UP      =1
uint8 CONNECT_DOWN    =2
uint8 CONNECT_FAILURE =3
```

#### 3.2. A Note on a Future Addition 

A potential limitation of the single-value enums above is that a component can only report one state per category at a time. Consider the `error_domain`: what happens if a hardware fault (`ERROR_HW`) immediately causes a communication failure (`ERROR_COMM`)? With the current design, the hardware driver must choose to (or is limited to) report only one.

A potential solution for this in a future iteration would be to define some enums as **bitfields**. This would involve assigning values as powers of 2, allowing multiple states to be combined using a bitwise `OR` operation.

For example, the `ErrorDomain` enum could be redefined as a bitmask:
```
# Example ErrorDomain as a bitfield
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

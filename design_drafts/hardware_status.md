# Proposal: Standardized Hardware Status Reporting

## Short description

Hey everyone, I'd like to kick off a discussion about a standardized way for hardware components in `ros2_control` to report their status.

Right now, if you're writing a hardware interface, how you report things like health, errors, or connectivity is pretty much up to you. This usually means everyone rolls their own custom messages. While that works for a single project, it makes it really tough to build generic, reusable tools on top of `ros2_control`. Imagine a diagnostic UI or a fleet management system that has to understand dozens of different ways to say "the battery is low" or "the motor has an over-temperature fault."

This proposal is a first-pass attempt at defining a generic `HardwareStatus` message. The main goal is to find a good balance between a structured, predictable format that tools can rely on, and the flexibility needed to report all the weird, wonderful, and specific details of different hardware.

This is very much a draft to get the conversation started, not a final solution!

## The Idea: Structured vs. Unstructured

The core idea is to split status reporting into two parts, likely within the same message structure.

1.  **Structured Status:** A set of common, standardized fields that cover about 80% of what most hardware needs to report. This part is meant to be machine-readable and used for automated monitoring and control logic.
2.  **Unstructured Status:** A flexible key-value bag for all the hardware-specific, non-standard stuff. This is primarily for diagnostics, logging, and human-readable UIs.

Here's a diagram I put together to visualize it:

![hardware_status](images/hardware_status.png)

Let's break it down.

---

### Part 1: Structured Status Messages

**Purpose:** This is the multi-purpose, standardized section. The fields are meant to be general enough to apply to many different types of hardware. The key here is that even if a piece of hardware doesn't use all the fields, it just populates the ones it can and uses `UNKNOWN` for the rest.

#### The Fields:

*   `string hardware_identifier`
    *   A unique name for the hardware component instance (e.g. - "left_wheel_motor").

*   `Health Status` (enum)
    *   A high-level summary of the component's health.
    *   `UNKNOWN`: The default state or if health cannot be determined.
    *   `OK`: All systems go.
    *   `DEGRADED`: Operational, but with reduced performance (e.g., a noisy sensor, a motor with reduced torque).
    *   `WARNING`: A non-critical issue has been detected that might lead to an error (e.g., temperature is rising).
    *   `ERROR`: A recoverable error has occurred. The component is not operational.
    *   `FATAL`: A non-recoverable error. The component is dead or requires a hard reset.

*   `Error Domain` (enum)
    *   If `Health Status` is `ERROR` or `FATAL`, this gives a clue as to *what kind* of error it is.
    *   `UNKNOWN`: Can't determine the source of the error.
    *   `NONE`: No error.
    *   `HW`: A physical hardware failure (e.g., a burnt-out motor driver, broken sensor).
    *   `FW`: A firmware-level fault or exception.
    *   `COMM`: A communication error (e.g., timeout, checksum failure).
    *   `POWER`: A power-related issue (e.g., undervoltage, overcurrent).

*   `Operational Status` (enum)
    *   Describes the current functional state of the device.
    *   `UNKNOWN`: Can't determine the operational state.
    *   `OFFLINE`: The component is powered but not initialized or ready to be used.
    *   `ONLINE`: The component is initialized and fully operational.
    *   `CALIBRATING`: The component is currently performing a self-calibration or startup routine.

*   `Connectivity Status` (enum)
    *   Specifically for networked devices or components with a clear connection state. Or maybe just any comm in general.
    *   `UNKNOWN`: Can't determine connectivity.
    *   `FAILURE`: A persistent connection failure.
    *   `UP`: The connection is established and active.
    *   `DOWN`: The connection is not established.

*   `Vendor Data` (struct)
    *   Static information about the hardware. This would likely be populated once at startup.
    *   `MANUFACTURER`: e.g., "Bosch"
    *   `MAKE`: e.g., "Lidar-XYZ-v2"
    *   `FW_VER`: e.g., "1.2.3"

#### The Question Marks (`?`)

As you can see in the diagram, I left a few boxes with question marks. This is where I'd love some feedback.

*   **Power Status?**: I went back and forth on this. Should power state (e.g., `ON`, `OFF`, `SLEEP`) be a standard field, or is it better handled in the `Operational Status` or even the unstructured section?
*   **The other `?` box**: I left one completely blank. Is there another critical, general-purpose status domain we're missing? Maybe a `Mode` status (e.g., `MANUAL`, `AUTOMATIC`, `SAFE_MODE`)? Or is that getting too specific?
---

### Part 2: Unstructured Status Messages

**Purpose:** It's for all the detailed, hardware-specific diagnostic information that doesn't fit neatly into the structured fields.

**Crucially**, this part should, at least in my opinion, be considered **for reporting purposes only**. High-level control logic should not be designed to parse these key-value pairs. It's for a human looking at a dashboard or a developer debugging a log file.

It would essentially be an array of key-value pairs (`diagnostic_msgs/KeyValue` ?).

**Example:**
For a custom robot controller board, the unstructured data might look like this:

```json
[
  {"key": "status_register",     "value": "0xFC4C1283"},
  {"key": "left_battery_health", "value": "Good"},
  {"key": "temperature_cpu",     "value": "72.5'C"},
  {"key": "imu_connection",      "value": "Lost"},
  {"key": "fan_operational",     "value": "False"}
]
```
This gives us the ultimate flexibility to report anything and everything without polluting the standardized part of the message.

## Open Questions & Points for Discussion

1.  Should this all be one big message (e.g., `control_msgs/HardwareStatus`)? Or should we publish structured and unstructured data on two separate topics (e.g., `/hardware_status` and `/hardware_diagnostics`)? A single message seems simpler, but separating them clearly divides their intended use.
2.  What are your thoughts on the `?` boxes in the diagram? What other top-level status categories are essential for a wide range of hardware?
3.  Is this whole approach overly complicated? I did get a slight feeling that maybe this is over categorised, and would defeat the purpose.

Looking forward to hearing what everyone thinks
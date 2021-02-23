# Definition of command interfaces for non-joint related information

## Motivation
Already multiple times, users raised the question about the handling of non-joint-related interfaces.
Such interfaces are not considerd during [design of the ros2_control URDF-tag structure](./components_architecture_and_urdf_examples.md).
The design proposes two possibilities to classify (group) the interfaces, using `<joint>` and `<sensor>` tags.
`<joint>`-tag groups the interfaces associated with the joints of physical robots and actuators.
They have command and state interfaces to set the goal values for hardware and read its current state.
`<sensor>`-tag groups multiple state interfaces describing, e.g., internal states of hardware.

Most modern robots, especially industrial manipulators, also have other physical ports to control external hardware.
The most usual ones are General Purpose Inputs/Outputs (GPIOs).
Although the strict definition considers only digital ports, the term GPIO in this document describes any (one-dimensional) input/output port of physical hardware, e.g., digital or analog input/output.

## Problem
To use GPIOs in the ros2_control framework, they have to be listed in the robot's URDF: inputs (state interfaces) under a `<sensor>`-tag and output (command and state interfaces) under a `<joint>`-tag.
This could lead to confusion since non-existing (virtual) joints have to be defined in the ros2_control part of the URDF structure. 
"Joint" is a well-defined term in robotics. Therefore this naming should be avoided, i.e., another keyword should be used describing GPIOs,  semantically separating them from the kinematics joints.

## Proposed solution
Use of keyword "gpio" when describing input and output ports of a robotic device that cannot be associated with any joint or sensor.
Parsing of `<gpio>-tag` is similar to this of a `<joint>`-tag having command and state interfaces.
The tag must have at least one `<command>`- or `<state>`-tag as a child.

The keyword "gpio" is chosen for its generality.
Although strictly used for digital signals, in this document describes any electrical analog, digital signal, or physical value.

The `<gpio>` tag can be used as a child of all three types of hardware interfaces, i.e., system, sensor, or actuator.
Semantically, they describe values that can not be associated with a joint or a sensor.

### Examples

1. Robot with multiple GPIO interfaces
   - RRBot System
   - Digital: 4 inputs and 2 outputs
   - Analog: 2 inputs and 1 output
   - Vacuum valve at the flange (on/off)

   ```
   <ros2_control name="RRBotSystemMutipleGPIOs" type="system">
     <hardware>
       <plugin>ros2_control_demo_hardware/RRBotSystemPositionOnlyHardware</plugin>
       <param name="example_param_hw_start_duration_sec">2.0</param>
       <param name="example_param_hw_stop_duration_sec">3.0</param>
       <param name="example_param_hw_slowdown">2.0</param>
     </hardware>
     <joint name="joint1">
       <command_interface name="position">
         <param name="min">-1</param>
         <param name="max">1</param>
       </command_interface>
       <state_interface name="position"/>
     </joint>
     <joint name="joint2">
       <command_interface name="position">
         <param name="min">-1</param>
         <param name="max">1</param>
       </command_interface>
       <state_interface name="position"/>
     </joint>
     <gpio name="flange_digital_IOs>
       <command_interface name="digital_output1"/>
       <state_interface name="digital_output1"/>    <!-- Needed to know current state of the output -->
       <command_interface name="digital_output2"/>
       <state_interface name="digital_output2"/>
       <state_interface name="digital_input1"/>
       <state_interface name="digital_input2"/>
     </gpio>
     <gpio name="flange_analog_IOs>
       <command_interface name="analog_output1"/>
       <state_interface name="analog_output1"/>    <!-- Needed to know current state of the output -->
       <state_interface name="analog_input1"/>
       <state_interface name="analog_input2"/>
     </gpio>
     <gpio name="flange_vacuum>
       <command_interface name="vacuum"/>
       <state_interface name="vacuum"/>    <!-- Needed to know current state of the output -->
     </gpio>
   </ros2_control>
   ```

2. Gripper with electrical and suction grasping possibilities
   - Multimodal gripper
   - 1-DoF parallel gripper
   - suction on/off
   
   ```
   <ros2_control name="MultimodalGripper" type="actuator">
    <hardware>
      <plugin>ros2_control_demo_hardware/MultimodalGripper</plugin>
    </hardware>
    <joint name="parallel_fingers">
      <command_interface name="position">
        <param name="min">0</param>
        <param name="max">100</param>
      </command_interface>
      <state_interface name="position"/>
    </joint>
    <gpio name="suction>
       <command_interface name="suction"/>
       <state_interface name="suction"/>    <!-- Needed to know current state of the output -->
     </gpio>
   </ros2_control>
   ```

3. Force-Torque-Sensor with temperature feedback and adjustable calibration
   - 2D FTS
   - Temperature feedback in °C
   - Choice between 3 calibration matrices, i.e., calibration ranges
   
   ```
   <ros2_control name="RRBotForceTorqueSensor2D" type="sensor">
     <hardware>
       <plugin>ros2_control_demo_hardware/ForceTorqueSensor2DHardware</plugin>
       <param name="example_param_read_for_sec">0.43</param>
     </hardware>
     <sensor name="tcp_fts_sensor">
       <state_interface name="fx"/>
       <state_interface name="tz"/>
       <param name="frame_id">kuka_tcp</param>
       <param name="fx_range">100</param>
       <param name="tz_range">100</param>
     </sensor>
     <sensor name="temp_feedback">
       <state_interface name="temperature"/>
     </sensor>
     <gpio name="calibration">
       <command_interface name="calibration_matrix_nr"/>
       <state_interface name="calibration_matrix_nr"/>
     </gpio>
   </ros2_control>
   ```
  

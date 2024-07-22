# Controller Interfaces Remapping

The idea of this approach is similar to the Topics, Parameters, Services and nodes in traditional ROS 2 approach. Remapping interfaces allows reusing the interfaces requested by the controller's ``state`` and ``command`` interface configuration and remap it to another interface instance. This helps to reduce the complexity of the controllers and the interfaces they can be used. 

For instance, a controller that uses ``effort`` as the interface type in combination with the ``joint`` names, however, there might be cases where the joints use a combination of ``prismatic`` and ``revolute`` joint types with the corresponding ``force`` and ``torque`` interfaces from the hardware. By allowing the remapping of interfaces, this exact controller can be used and these ``force`` and ``torque`` interfaces are remapped to the ``effort`` interfaces of the controller. Thereby, reduing the complexity at the controller design to support various interfaces.

The proposed remapping approach is for each of the controllers to be able to define the ``remap`` parameter namespace and define the ``state`` and ``command`` interfaces. The following would be an example of the controller that remaps its interfaces

```yaml
arm_controller:
  ros__parameters:
    type: joint_trajectory_controller/JointTrajectoryController
    joints:
      - arm_1_joint
      - arm_2_joint
      - arm_3_joint
      - arm_4_joint
      - arm_5_joint
      - arm_6_joint
      - arm_7_joint
    command_interfaces:
      - effort
    state_interfaces:
      - position
      - velocity
    remap:
        state_interfaces:
            arm_2_joint/position: "arm_2_joint/absolute_encoder/position"
        command_interfaces:
            arm_1_joint/effort: "arm_1_joint/torque"
            arm_2_joint/effort: "arm_2_joint/torque"
            arm_3_joint/effort: "arm_3_joint/force"
            arm_4_joint/effort: "arm_4_joint/torque"
            arm_5_joint/effort: "arm_5_joint/force"
            arm_6_joint/effort: "arm_6_joint/force"
            arm_7_joint/effort: "arm_7_joint/torque"

```

The idea is to just change the multiple ``state_interface_configuration`` and ``command_interface_configuration`` method called inside the Controller Manager with a new method that just shows the remapped part, this way it is very transparent to the user as no modifications are really needed by the controller itself. It is also better to print the remappings at the configure stage, so that the user is aware of any possible issues if it may cause. I believe the usecases from this approach is beyond the above explained usecase, and simple design of the controllers does go in favor with this approach.
# ros2_control profiles

When a system has multiple controllers or a chain of controllers then switching between different control modes is quite a hassle as all the respective controllers needs to be activated and deactivated. In order to handle those cases, I propose something called profiles in the ros2_control framework which is a separate package or a separate python node that handles these transmission. Every package is free to define their own profiles and we will be getting that information at runtime using the [ament_resource_index](https://github.com/ament/ament_cmake/blob/master/ament_cmake_core/doc/resource_index.md#implementation). This allows flexibility and modularity for the design

We would need a new service and topic message types, for instance as below:

```
controller_manager_msgs/Profile

string name
string[] controllers_list
string metadata
```

```
#controller_manager_msgs/Profiles

controller_manager_msgs/Profile[] profiles
```

The above `controller_manager_msgs/Profiles` can be used to get the list the profiles and also to publish the information of active profiles from the profile manager and to know the information of the available profiles, which will be depending on the configured controllers.

```
# controller_manager_msgs/SwitchProfiles

string[] activate_profiles
string[] deactivate_profiles
-----
string message
bool success
```

We will need a profile manager that exposes the service `~/switch_profiles`. The profile manager is responsible to read the assets and then expose the service, so that the user can switch profiles as he intended

An example controller profile yaml will look like:

Example 1:
```yaml
cartesian_impedance_control:
    controllers_list : ["cartesian_impedance_controller", "arm_left_1_joint_torque_controller", "arm_left_2_joint_torque_controller", "arm_left_3_joint_torque_controller", "arm_left_4_joint_torque_controller", "arm_left_5_joint_torque_controller", "arm_left_6_joint_torque_controller", "arm_left_7_joint_torque_controller"]
    metadata:
        input_type: topic
        input_topic_name: "/cartesian_impedance_controller/reference"
        description: "A profile to set the robot in cartesian impedance control mode" 
```

Example 2:

```yaml
joints_torque_controllers:
    controllers_list : ["arm_left_1_joint_torque_controller", "arm_left_2_joint_torque_controller", "arm_left_3_joint_torque_controller", "arm_left_4_joint_torque_controller", "arm_left_5_joint_torque_controller", "arm_left_6_joint_torque_controller", "arm_left_7_joint_torque_controller"]
    metadata:
        description: "A profile to start the torque controller for all the joints"

linear_mpc_controller:
    controllers_list: ["linear_mpc_controller"]
    metadata:
        description: "A profile to start the MPC controller"
```
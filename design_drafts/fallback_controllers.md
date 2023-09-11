# Fallback Controllers

## Motivation

Right now, we don't have a way to manage controllers when they fail to perform their task. We might want the robot to perform a different control scheme for the safety of the robot. The goal of the fallback controllers is to take control of the joints, when the main controller fails.

### Example 1
A walking controller for a humanoid is also responsible to maintain the balance of the robot along with footstep execution, however there might be cases where the robot might fall after few steps due to miscalibration or due flexibility in some joints or it could be a different reason. In this case, we might want the robot to go to a safe position and be compliant enough for the impact, if not it might damage some joints or sensors onboard.

### Example 2
A mobile manipulator is performing a trajectory and hits an obstacle and continue to apply force. In this case, we might want to switch to be more compliant mode, so that it doesn't damage the joints of the robot.

## Current implementation

The current implementation fetches the return type from the current controller update cycle and then decides the return type status of the controller_manager's update cycle.

```c++
auto controller_ret = loaded_controller.c->update(
  time, (controller_update_factor != 1u)
          ? rclcpp::Duration::from_seconds(1.0 / controller_update_rate)
          : period);

if (controller_ret != controller_interface::return_type::OK)
{
  ret = controller_ret;
}
```

## Implementation proposal

Currently, we have a way to handle hardware errors in both `read` and `write` methods by deactivating the controller, but not when the controller returns error. The proposal focuses on the handling of the controller return status properly along with activating new set of controllers, known as fallback controllers.

### ControllerInfo with fallback controllers list

When loading the controller by retrieving its info from the paramserver, along with it we also can get the list of fallback controllers. This list of fallback controllers can be stored in the `ControllerInfo`.

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint1_position_controller:
      type: fallback_controller/FallBackController

    joint2_position_controller:
      type: fallback_controller/FallBackController

    position_controller:
      type: forward_command_controller/ForwardCommandController
      fallback_controllers: ["joint1_position_controller", "joint2_position_controller"]
```

``` c++
struct ControllerInfo
{
  /// Controller name.
  std::string name;

  /// Controller type.
  std::string type;

  /// List of claimed interfaces by the controller.
  std::vector<std::string> claimed_interfaces;

  std::vector<std::string> fallback_controllers;
};
```

### Activating the controller with fallback controllers

When trying to activate the controller with fallback controllers, the following checks are needed:

* All the corresponding fallback controllers should be already configured
* Fallback controllers list should be able to activate without any dependency on other controllers (in case of chaining with different controller) outside the list
* All the main controller command interfaces should be present in the fallback controllers command interfaces

If any of the above checks fail, then activation of the main controller fails. The user will always be able to run them without configuring the fallback controller. 


### The `update()` method

Deactivate the controllers that have return type other than OK, and then activate the already configured fallback controllers from the list, so in the next update cycle, the fallback controllers can act.

```c++
std::vector<std::string> stop_request = {};
std::vector<std::string> start_request = {};
...
auto controller_ret = loaded_controller.c->update(
  time, (controller_update_factor != 1u)
          ? rclcpp::Duration::from_seconds(1.0 / controller_update_rate)
          : period);

if (controller_ret != controller_interface::return_type::OK)
{
  stop_request.push_back(loaded_controller.info.name);
  cont auto &fallback_controllers_list = loaded_controller.info.fallback_controllers;
  start_request.insert(start_request.end(), fallback_controllers_list.begin(), fallback_controllers_list.end());
  ret = controller_ret;
}
...
if(!stop_request.empty())
  deactivate_controllers(rt_controller_list, stop_request);
if(!start_request.empty())
  activate_controllers(rt_controller_list, start_request);
```

## Closing remarks

- In future, we should be able to disable all the preceeding control chains prior to the controller that returns other than OK, inorder integrate well with controller chaining. For the initial implementation, it can be limited to the non-chainable controllers.

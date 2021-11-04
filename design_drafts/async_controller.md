# Asynchronous Controllers in ros2_control

## What is an asynchronous controller?

An asynchronous controller is a controller that for some reason cannot (or we don’t want to) perform the operations needed in an update() call.
For instance if ros control is running at 100Hz, the sum of the execution time of all controllers update() calls must be below 10ms. If a controller requires 15ms it cannot be executed synchronously without affecting the whole ros control update rate.

## Abstract implementation description
Create an AsyncControllerWrapper class that can wrap any Controller without modifying it and makes it run asynchronously seamlessly.
It will receive a Controller and store it, and act as an intermediary between the ros_control loop and the Controller.
On the update() call, the AsyncControllerWrapper will read data from the interfaces, store it in an intermediate object, and write data from another intermediate object to the interfaces.
The wrapped Controller, running on a separate thread, will read and write from these intermediate objects.
This data must be protected by mutexes or similar.
When new state data is available, the AsyncControllerWrapper will notify the wrapped controller so it is processed on its own thread.

## Implementation guidelines:

### Concurrency risks
The main risk is handling concurrency, since we want to wrap any controller, which was probably not implemented to support concurrent calls to some of its methods.
The only method of the wrapped controller that we’ll run in a separate thread, is update().
The rest of the method calls will be forwarded from the AsyncControllerWrapper to the wrapped controller in the main thread.
We must make sure that the asynchronous update() is not being executed when forwarding other methods.

### StateInterfaces and CommandInterfaces
The second risk we want to control, is that our asynchronous controller has access to State and Command Interfaces, and uses them from its own thread, causing for instance concurrent write of a CommandInterface while the Actuator is reading it.

To avoid this, we can create intermediate Interfaces, stored in the AsyncControllerWrapper, that act as a bridge between the wrapped controller and the real State and Command Interfaces.
This bridge should allow thread safe data storage, and non blocking operations on the ros_control thread. Some containers for this purpose exist in: https://github.com/ros-controls/realtime_tools/tree/ros2_devel/include/realtime_tools

These interfaces should be created on the assign_interfaces method, and removed on the release_interfaces method. But at the moment those methods are not virtual.
I’ll assume they will be changed to virtual, but if that cannot be done for ABI compatibility issues, the same could be done in the on_activate and on_deactivate, although it might be less correct.

Sample implementation (not tested), we create our own State and Command interfaces, and assign them to the wrapped controller.

```c++
using ControllerInterface = controller_interface::ControllerInterface;
class AsyncControllerWrapper : public controller_interface::ControllerInterface
{
public:
 AsyncControllerWrapper(std::shared_ptr<ControllerInterface> wrapped_controller);
 void assign_interfaces(
   std::vector<hardware_interface::LoanedCommandInterface> && command_interfaces,
   std::vector<hardware_interface::LoanedStateInterface> && state_interfaces)
 {
   std::vector<hardware_interface::LoanedCommandInterface> loaned_bridge_command_interfaces_;
   for (const auto & ci : command_interfaces) {
     bridge_command_data_.push_back(ci.get_value());
     bridge_command_interfaces_.emplace_back(
       ci.get_name(), ci.get_interface_name(),
       &bridge_command_data_.back());
     loaned_bridge_command_interfaces_.emplace_back(bridge_command_interfaces_.back());
   }
   std::vector<hardware_interface::LoanedStateInterface> loaned_bridge_state_interfaces_;

   for (const auto & si : state_interfaces) {
     bridge_state_data_.push_back(si.get_value());
     bridge_state_interfaces_.emplace_back(
       hardware_interface::StateInterface(
         si.get_name(), si.get_interface_name(),
         &bridge_state_data_.back()));
     loaned_bridge_command_interfaces_.emplace_back(bridge_command_interfaces_.back());
   }
   wrapped_controller_->assign_interfaces(
     std::move(loaned_bridge_command_interfaces_),
     std::move(loaned_bridge_state_interfaces_));
 }

 void release_interfaces()
 {
   wrapped_controller_->release_interfaces();
 }

 virtual controller_interface::return_type update() override
 {
   // Caution, this is just an incomplete example on how to copy the data from the real interfaces
   // to our bridge interfaces
   // This access must be protected so it does not concurrently with the asynchronous update()

   for (size_t i = 0; i < state_interfaces_.size(); ++i) {
     bridge_state_data_[i] = state_interfaces_[i].get_value();
   }

   for (size_t i = 0; i < bridge_command_interfaces_.size(); ++i) {
     bridge_command_interfaces_[i].set_value(bridge_command_data_[i]);
   }
 }

protected:
 std::shared_ptr<ControllerInterface> wrapped_controller_;
 std::vector<hardware_interface::CommandInterface> bridge_command_interfaces_;
 std::vector<double> bridge_command_data_;
 std::vector<hardware_interface::StateInterface> bridge_state_interfaces_;
 std::vector<double> bridge_state_data_;
};
```

### The `update()` method
The `update()` method needs to:
Write commands from the bridge command interfaces to the real command interfaces.
Read real state interfaces and write them to the bridge state interfaces.
Notify the thread that it has new data to run.

And do all this ensuring:
That the thread is not running and actively reading/writing from the data at the same time
That the caller thread is not blocked on a mutex waiting for the asynchronous update() to end (as this defeats the purpose of asynchronous controller).


#### Implementation considerations
For the asynchronous `update()` calls on the wrapped controller, avoid using std::async as it would create a new thread on each call. This can have a significant overhead on the wrapper.
A better idea is probably to create a thread, that we wake up when there’s new data.
For waking up a thread, std::condition_variable is a good solution, although it is not real time safe. For the moment it can be implemented like this and in the future, when the rest of the code is made RT safe, this can be reviewed.
Alternatively the wrapped controller could run in a non-blocking periodic thread at a certain rate.

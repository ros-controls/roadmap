# Loading Controller as Components

## Components in a Nutshell

ROS2 Eloquent introduced the concept of components.
A component represents a node which can be conveniently loaded into a generic components container and so being executed without the node to compile the node in its own executable.
Throughout the rest of this document, the word `node` is equivalent to `component`.
The component container (a.k.a `ComponentManager`) provides functionalities such as services and command line interfaces to dynamically load and unload components.
It takes ownership over the loaded components and can thus control their execution.

That should be enough background for the scope of this document.
For more details see https://index.ros.org/doc/ros2/Tutorials/Composition/

## Controller Manager as a Component Manager

The `ComponentManager` is in a sense very similar to the `ControllerManager` in which the controller manager holds the same logic of dynamically loading and unloading controllers.
We therefore propose to leverage the functionality of the `rclcpp_components` within the controller manager, which provides all necessary services and command line tools needed to load components.

### Type Safety of Components

The component loading functionality differs from the `rclcpp_components` in a way that the controller manager has to make sure to load only components which adhere to the `ControllerInterface` and not load any arbitrary nodes.
The current component container interface does not allow to filter for specific types and has to be modified accordingly.
The proposed design lets the controller manager being a child class of the component manager and gives so the change to override the appropriate functions to guarantee a strict type safety.
An alternative, more speculative, approach could be to modify the original component manager to support the type filtering.

### Implications for Controllers

In order for a node to qualify as a component, it really only has to inherit from a _node_-like such as `rclcpp::Node` or `rclcpp_lifecycle::LifecycleNode`.
It further requires a constructor which accepts a single `const rclcpp::NodeOptions & options` argument which is then passed to the parent constructor in order to parse things such parameters correctly.
At last, each controller has to be explicitly exported via `cmake` and so registered in the `ament_resource_index` to be found at runtime.

All these requirements are easily being fulfilled by a controller and thus doesn't require disruptive changes to its current design.

## Extended Functionality of the Controller Manager

The `ComponentManager` really only loads and unloads components at runtime and provides the necessary services for doing so.
The functionality of the controller manager exceeds these by managing hardware resources and as well as the life cycle of individual controllers.

### Life Cycle Management of Controllers

Each loaded controller inherits from the `ControllerInterface` and therefore from a `LifecycleNode`.
The responsibilities for the controller manager differs from the component manager such that loaded controllers have to be configured and activated appropriately.
It further has to react to life cycle events in case controllers run into failure behaviors and might unload the failing controller.

### Resource Management

Controllers have the ability to claim hardware resources.
These resources might be potentially in conflict when multiple controllers try to acquire ownership of these resources concurrently.
It shall further be the responsibility of the controller manager to react appropriately to these conflicts and unload either one or all conflicting controllers.

### Execution Management of Controllers

One important aspect of the controller manager is its execution model which is responsible for in which order the loaded controllers are being processed.
The component manager in its current implementation accepts a pointer to an `rclcpp::Executor` which could be leverage to provide an executor tailored for the controller manager.
Details on how this execution management can look like is outside of the scope of this document and shall be discussed in its dedicated design document.
The point here being made is solely that a component manager must not interfere with the execution management applied in the controller manager.

# Roadmap for ros_control in ROS2

Please note that these milestones/tasks are not ordered.

| Milestones  | ROS distribution |
| ------------- | ------------- |
| Modernization of existing ros_control framework and controllers' code  | ROS Melodic, ROS Noetic  |
| Full ros_control features with ROS2 interfaces, ported over  | ROS2 Foxy  |
| Main controllers for MoveIt, navigation and Gazebo integration made available. | ROS2 Foxy |
| Integrate a new ros2_control verb into the ros2cli instead of the current CLI service interface. Examples: “ros2 control switch ...” , “ros2 control load ...” | ROS2 Foxy |
| Adopt ROS2 managed node concept for lifecycle handling | ROS2 G |
| Achieve chaining controllers through ROS2 topics | ROS2 G |
| Hard-real time guarantees | ROS2 G |
| Per-controller configurable update loop | ROS2 G |
| Revise nomenclature, read-only “controllers” should be sensors. Anything declared as a sensor has to be enforced to be read-only. | ROS2 G |
| ros_control’s one-stop documentation portal at https://index.ros.org/doc/ros2/. Hosts improved documentation and tutorials in a single place. Remove dangling documentation from other sources. | ROS2 G |
| Common emergency stop handler | ROS2 G |
| Port all other controllers over. | ROS2 G |


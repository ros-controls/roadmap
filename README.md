# Roadmap for ros_control in ROS2

Please note that these milestones/tasks are not ordered.

| Milestones  | ROS distribution | Related issues, PRs |
| ------------- | ------------- | ------------- |
| Modernization of existing ros_control framework and controllers' code  | ROS Melodic, ROS Noetic  | [ros_control#403](https://github.com/ros-controls/ros_control/issues/403), [ros_control#407](https://github.com/ros-controls/ros_control/issues/407), [ros_controllers#380](https://github.com/ros-controls/ros_controllers/pull/380) |
| Full ros_control features with ROS2 interfaces, ported over  | ROS2 Foxy  | [ros2_control#21](https://github.com/ros-controls/ros2_control/issues/21), [ros2_control#23](https://github.com/ros-controls/ros2_control/issues/23), [ros2_control#24](https://github.com/ros-controls/ros2_control/issues/24), [ros2_control#26](https://github.com/ros-controls/ros2_control/issues/26), [ros2_control#41](https://github.com/ros-controls/ros2_control/issues/41), [ros2_control#33](https://github.com/ros-controls/ros2_control/issues/33) |
| Main controllers for MoveIt, navigation and Gazebo integration made available. | ROS2 Foxy | [ros2_control#25](https://github.com/ros-controls/ros2_control/issues/25), [ros2_controllers#9](https://github.com/ros-controls/ros2_controllers/issues/9) |
| Integrate a new ros2_control verb into the ros2cli instead of the current CLI service interface. Examples: “ros2 control switch ...” , “ros2 control load ...” | ROS2 Foxy | [ros2_control#29](https://github.com/ros-controls/ros2_control/issues/29) |
| Adopt ROS2 managed node concept for lifecycle handling | ROS2 G | [ros2_control#43](https://github.com/ros-controls/ros2_control/issues/43) |
| Achieve chaining controllers through ROS2 topics | ROS2 G | [ros2_control#30](https://github.com/ros-controls/ros2_control/issues/30) |
| Hard-real time guarantees | ROS2 G | |
| Per-controller configurable update loop | ROS2 G | [ros2_control#31](https://github.com/ros-controls/ros2_control/issues/31) |
| Revise nomenclature, read-only “controllers” should be sensors. Anything declared as a sensor has to be enforced to be read-only. | ROS2 G | |
| ros_control’s one-stop documentation portal at https://index.ros.org/doc/ros2/. Hosts improved documentation and tutorials in a single place. Remove dangling documentation from other sources. | ROS2 G | [ros2_control#22](https://github.com/ros-controls/ros2_control/issues/22) |
| Common emergency stop handler | ROS2 G | |
| Port all other controllers over. | ROS2 G | |

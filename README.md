# Roadmap for ros_control in ROS2

Please note that these milestones/tasks are not ordered.

| Milestones  | ROS distribution | Targeted Time | Related issues, PRs |
| ------------- | ------------- | ------------- | ------------- |
| **Ongoing maintenance and improvements to ros_control in ROS1** | | | |
| Modernization of existing ros_control framework and controllers' code  | ROS Melodic, ROS Noetic  | M5 2020 | [ros_control#403](https://github.com/ros-controls/ros_control/issues/403), [ros_control#407](https://github.com/ros-controls/ros_control/issues/407), [ros_controllers#380](https://github.com/ros-controls/ros_controllers/pull/380), [ros_control#420](https://github.com/ros-controls/ros_control/issues/420), [ros_control#449](https://github.com/ros-controls/ros_control/issues/449) [ros_controllers#513 general package dependency cleanup](https://github.com/ros-controls/ros_controllers/issues/513)|
| **Support for ros_control features in ROS2** | | | |
| Feature parity with ROS1 ros_control  | ROS2 Foxy | M12 2020 | [ros2_control#21](https://github.com/ros-controls/ros2_control/issues/21), [ros2_control#23](https://github.com/ros-controls/ros2_control/issues/23), [ros2_control#24](https://github.com/ros-controls/ros2_control/issues/24), [ros2_control#26](https://github.com/ros-controls/ros2_control/issues/26), [ros2_control#41](https://github.com/ros-controls/ros2_control/issues/41), [ros2_control#33](https://github.com/ros-controls/ros2_control/issues/33), [flexible joint states github project](https://github.com/orgs/ros-controls/projects/2) |
| Main controllers for MoveIt, navigation and Gazebo integration made available. | ROS2 Foxy | M12 2020 | [ros2_control#25](https://github.com/ros-controls/ros2_control/issues/25), [ros2_controllers#9](https://github.com/ros-controls/ros2_controllers/issues/9) |
| **New features in ROS2** | | | |
| Make joint interfaces more flexible, allow user-defined joint interfaces without C++ types to lock them in | ROS2 Foxy | M8 2020| [flexible joint states github project](https://github.com/orgs/ros-controls/projects/2),  [design doc](https://github.com/ros-controls/roadmap/blob/master/design_drafts/flexible_joint_states_msg.md), [ros2_control joint interfaces](https://github.com/ros-controls/ros2_control/pull/223/), [transmission handling project](https://github.com/orgs/ros-controls/projects/4), [ros2_control#134](https://github.com/ros-controls/ros2_control/pull/134)|
| Adopt ROS2 managed node concept for lifecycle handling | ROS2 Foxy | M5 2020 | [ros2_control#43](https://github.com/ros-controls/ros2_control/issues/43) |
| Integrate a new ros2_control verb into the ros2cli instead of the current CLI service interface. Examples: “ros2 control switch ...” , “ros2 control load ...” | ROS2 Foxy | M8 2020 | [ros2_control#29](https://github.com/ros-controls/ros2_control/issues/29) |
| Revise framework with component-based design | ROS2 Foxy | M11 2020 | [ros2_control#147](https://github.com/ros-controls/ros2_control/issues/147), [ros2_control#164](https://github.com/ros-controls/ros2_control/issues/164), [ros2_control#164](https://github.com/ros-controls/ros2_control/issues/164), [ros2_control#201](https://github.com/ros-controls/ros2_control/issues/201), [ros2_control#203](https://github.com/ros-controls/ros2_control/issues/203), [ros2_control#207](https://github.com/ros-controls/ros2_control/issues/207), [ros2_control#224](https://github.com/ros-controls/ros2_control/issues/224), [ros2_control#216](https://github.com/ros-controls/ros2_control/issues/216), [ros2_control#234](https://github.com/ros-controls/ros2_control/issues/234), [ros2_control#236](https://github.com/ros-controls/ros2_control/issues/236) |
| Revise nomenclature, read-only “controllers” should be sensors. Anything declared as a sensor has to be enforced to be read-only. | ROS2 Foxy | M12 2020| [ros2_controllers#110](https://github.com/ros-controls/ros2_controllers/issues/110) |
| Achieve chaining controllers through ROS2 topics | ROS2 Galactic | M12 2020 | [ros2_control#30](https://github.com/ros-controls/ros2_control/issues/30) |
| Hard-real time guarantees | ROS2 Galactic | M1 2021 |
| Per-controller configurable update loop | ROS2 Galactic | M1 2021 | [ros2_control#31](https://github.com/ros-controls/ros2_control/issues/31) |
| Common emergency stop handler | ROS2 Galactic | M2 2021 | |
| Port all other controllers over. | ROS2 Galactic | M2 2021 |
| **Testing and validation on different robot platforms** | | | |
| Real robot tests | ROS2 Foxy | M12 2020 | |
| Tutorials with simulated robots? | ROS2 Foxy | M12 2020 | [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos), [demos github project](https://github.com/orgs/ros-controls/projects/1) |
| ROS2 Quality Level | ROS2 F-G | M1 2021 | [roadmap#20](https://github.com/ros-controls/roadmap/issues/20) |
| **Documentation work for ROS1 and ROS2** | | | |
| Improve existing ROS1 documentation on ROS wiki | ROS2 Melodic, ROS Noetic | M2 2021 | [ros_control#457](https://github.com/ros-controls/ros_control/pull/457), [ros_control#433](https://github.com/ros-controls/ros_control/pull/433) |
| ros_control’s one-stop documentation portal at https://index.ros.org/doc/ros2/. Hosts improved documentation and tutorials in a single place. Remove dangling documentation from other sources. | ROS2 Galactic | M2 2021 | [ros2_control#22](https://github.com/ros-controls/ros2_control/issues/22) |
| Additional tutorials for developing RobotHW | ROS2 Galactic | M2 2021 | |


# Multi-Time Joint Trajectory Controller

## Use Case and JTC Limitations

A client project currently uses a 6-axis Joint Trajectory Controller (JTC) in tandem with some modifications to control the cartesian trajectory of an omnidirectional robot (which is able to translate and rotate in 3D).
The client controls a subset of the 6 axes at any given time (sometimes all of them), and some axes will be doing different things at any given time (i.e., some axes will have closed-loop control targeting a position, some will have velocity control, and some will be uncontrolled).
As such, each axis may have a different desired time to reach its target.
Because of this, the current implementation has some logic to change the goal time of the trajectory points such that the robot reaches each desired state at its desired time.
However, this logic is complex and error-prone since it has to be implemented at the call site rather than in the controller itself.
As a result, I propose implementing a multi-time variant of the joint trajectory controller.

This Multi-Time Joint Trajectory Controller (MTJTC) will operate in a similar manner to the regular JTC, but will allow each joint trajectory point to have multiple end times, instead of just the one.

## Alternatives Considered

Before proposing to implement this controller, several other alternatives were considered.

The most obvious option was considered first: having a single JTC for each axis. While this option would allow for much of the desired behavior, it is both cumbersome to manage (in our case requiring 6 separate JTCs), difficult to chain (as we currently have a single controller chained to the modified JTC we are using), and does not allow for synchronization of different axes when desired (sometimes multiple axes *should* be synced, they are not always independently controlled).

Another possible way to allow for the desired behavior was to add interfaces to the JTC such that it itself can take the multi-time joint trajectory points as inputs and use them as either a regular JTC or the MTJTC based on configuration. This was ruled out primarily for software engineering reasons. The current JTC already has a great deal of functionality, and adding another function would add a great deal of complexity to every facet of the code, requiring additional configuration, subscriptions, and greatly impacting code maintainability. Additionally, it would violate the single responsibility principle. The JTC, as the name implies, is designed to follow joint space trajectories. The current desired usage is definitely outside this original scope, as a joint trajectory with multiple times at each point really does not make sense in the context of a robotic arm, for example. To not impact the JTC's maintainability and to uphold the single responsibility principle, creating an entirely new controller makes more sense than to greatly modify the existing JTC.

Several other options were considered, though the above two were the most viable alternatives. Among these other options was contributing a package to automatically manage multiple JTCs. While this would capture most of the functionality, it would not be able to synchronize multiple of them when desired without sophisticated logic. Another less viable option that was considered was chaining a controller to translate the JTC's output times (essentially modifying the output after the fact to have multiple target times), allowing for time scaling per-axis within the JTC.

## Implementation

While the specifics will become more clear as implementation progresses, the general idea is not to inherit from the JTC but rather reuse relevant parts of it. This is because inheritance would bring in a lot of unnecessary variables that would not be used, though will unfortunately cause some code duplication. Overall, though, the MTJTC will end up being a controller with similar logic but will be distinct in most internal logic enough to warrant avoiding inheritance.

Currently, in an internal prototype, we utilize modified versions of the `MultiDOFJointTrajectoryPoint`, `JointTrajectory`, and `JointTrajectoryPoint` messages. These are called `MultiDOFMultiTimeJointTrajectoryPoint`, `MultiTimeJointTrajectory`, and `MultiTimeJointTrajectoryPoint`. Currently, these live in an internal fork of `control_msgs` (and would be included when the controller is ready to be contributed), but I am open to suggestions about these messages, too.

## Desired Results

Ultimately, the most important goal is to create a controller that can be configured to either synchronize or not synchronize any of its axes, and can accept goals with different end times per axis. This would allow for the client's desired use case and enable a great deal of functionality.
Ultimately, there may be other valid solutions to this problem (or better names for the controller), and before committing to this approach, we want to be sure that this idea is a sensible approach and will result in a controller that has use cases in the ros2_control community at large.
The client wants to contribute this controller back upstream to the open source community, so this document serves as a general proposal and is open for feedback.


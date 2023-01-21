## Implementation of Asynchronous Controllers


#### Short description

Asynchronous controllers are an upcoming addition to the ros2_control framework. As the name suggests, they make it possible to run the update calls of these controllers on a separate thread. Asynchronous components are omitted here for the sake of clarity, but they follow a very similar structure.
Since the intention was to support real-time systems, the standard library's std::atomic interface was used, and due to the inherent difficulty of writing and understanding concurrent lock-free programs, a brief glance at the code might not be enough to figure out if something doesn't work as intended.

For that reason, the purpose of this design draft is to explain how the synchronization works between the Controller Manager's thread and an asynchronous controllers, now to make it easier for others to review the PR of the feature, and later to let the users know if the current implementation is suitable for their use case.

#### Implementation

The controllers running asynchronously are entirely independent of each other, which means that no data sharing occurs between them. Therefore synchronization happens between the Controller Manager's thread and an async controller - so in the following drawings only these two threads will be visualized.

The critical sections are:

* The state interface writes from the Controller Manager's read function
* The command interface writes of the async controller's update function

The former has to occur and be seen by the async thread before the update, while the latter, ideally, has to finish and be seen before the controller manager's write.

![async_controller](https://user-images.githubusercontent.com/25421074/212979058-5dad88c1-d940-4bc7-ac6f-103711de7210.png)

The above diagram shows four different concurrency cases to account for:

1. The state interfaces aren't written yet by the cm read. In this case, we can be sure that the release flag isn't set, thus the acquire load on the other side will return false - so we'll skip the update and the asynchronous controller will sleep until the next iteration. This works as intended.

2. The state interfaces are written in the cm read function, and the release flag is set. It can happen that the state interfaces are visible on the controller's side, but the flag isn't - in this case, we'll do the same as the first scenario. On the other hand, if the flag is set, then the thread's acquire load will return true if it isn't currently sleeping, and the respective orderings guarantee that the async update will be able to work with the latest state interface values. This is fine too.

3. After async update, a release store is made to signal that the command interfaces are written. Again, the release ordering guarantees that if the store is visible on the other thread, then so are the command interfaces. If the acquire load of this variable is true right before the cm write function, then we're again working with the latest values, which is the intended outcome.

4. If the flag isn't visible, and the load return false, it means that the asynchronous update isn't finished before the next write. This is not ideal, since it means that two threads might access the same variable in the same time, leading to a potential data race. Currently, if this happens, only a warning is given to the user, and we continue with the write function as if nothing happened. In practice, probably nothing dangerous will happen, since the aligned loads and stores of plain pointers (similarly to ints and bools) are atomic on most CPUs, and this can only occur if the async update is slower than all synchronous updates combined, but even then, this is an issue and should be handled differently.

Here are are few options that could alleviate the problem:

* At the end of the cm update call, wait for the asynchronous controller to finish. As locking is out of the question, busy waiting could also be tried. However, this could potentially defeat the purpose of using an asynchronous controller.

* Handles should work with atomic primitives, if they are added in the future. The set_value() and get_value() functions could be relaxed stores and loads of the value. This ensures that we're working on tear-free variables without relying on platform specific implementations, with minimal overhead compared to a normal, non-atomic variable.

* There could be a mechanism to check which interfaces are used by which controllers and components. This could let us know if the async update is finished or not, and simply skip the following write, or address it in another manner.

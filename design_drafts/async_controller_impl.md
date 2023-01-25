## Implementation of Asynchronous Controllers


#### Short description

Asynchronous controllers are an upcoming addition to the ros2_control framework. As the name suggests, they make it possible to run the update calls of these controllers on a separate thread.
For synchronization, the standard library's std::atomic interface was used.

The purpose of this design draft is to explain how the synchronization works between the Controller Manager's thread and an asynchronous controller.
The asynchronous controllers' threads are entirely independent of each other, which means that no data sharing occurs between them. Synchronization happens only between the Controller Manager's thread and an async controller - so in the following drawings only these two threads will be visualized, even though it's possible to use multiple async controllers at once.


#### Implementation


The critical sections are:

* The state interface writes from the Controller Manager's read function
* The command interface writes of the async controller's update function

The former has to occur and be seen by the async thread before the update, while the latter, ideally, has to finish and be seen before the controller manager's write.
The implementation is based on the synchronization between the atomic operations' release and acquire orderings. In short, no operations sequenced before a release-ordered atomic store may move beyond the release store itself. Acquire is the opposite; no operation sequenced after the acquire load may move before the load itself. These guarantees are necessary because even though operation A is sequenced before operation B, the result of operation A might be visible later in the other thread than the result of operation B.

In practice, this is how it would work:

```c++
void cm_thread_read() {
  int state_if_one = 12; // may be visible after the store to state_if_two but not the atomic flag (release)
  int state_if_two = 13; // may be visible after the store to state_if_one but not the atomic flag (release)
  state_interfaces_ready.store(true, std::memory_order_release); // std::atomic<bool>, if the controller thread sees the result of this store, 
}

void controller_thread_update() {
  if (state_interfaces_ready.load(std::memory_order_acquire) { // only use the state_if values if the state_interfaces_ready flag is true
      int local_var_used_by_update = state_if_one; // may be reordered with the load in the next operation, but not the atomic flag (acquire)
      int other_local_var_used_by_update = state_if_two; // may be reordered with the load in the previous operation, but not the atomic flag (acquire)
  }
}

```

<br>

The code of the current implementation is a bit different (e.g., instead of loads atomic exchange operations are used to set the flag to false immediately), but also uses this principle. The next diagram shows the complete architecture, along with four different concurrency cases to account for.

<br>

![async_controller](https://user-images.githubusercontent.com/25421074/214663116-b0de8240-7efd-4ec4-bbc2-5a87d76430ca.png)


1. The state interfaces aren't written yet by the CM read. In this case, we can be sure that the release flag isn't set, thus the acquire load on the other side will return false - so we'll skip the update and the asynchronous controller will sleep until the next iteration.

2. The state interfaces are written in the cm read function, and the release flag is set. It can happen that the state interfaces are visible on the controller's side, but the flag isn't - in this case, we'll do the same as the first scenario. On the other hand, if the flag is set, then the thread's acquire load will return true if it isn't currently sleeping, and the respective orderings guarantee that the async update will be able to work with the latest state interface values. This is fine too.

3. After async update, a release store is made to signal that the command interfaces are written. Again, the release ordering guarantees that if the store is visible on the other thread, then so are the command interfaces. If the acquire load of this variable is true right before the cm write function, then we're again working with the latest values, which is the intended outcome.

4. If the flag isn't visible, and the load return false, it means that the asynchronous update isn't finished before the next CM write. This is not ideal, since it means that two threads might access the same variable in the same time, leading to a potential race condition. Currently, if this happens, only a warning is given to the user, and we continue with the write function as if nothing happened. In practice, probably nothing dangerous will happen, since the aligned loads and stores of plain pointers (similarly to ints and bools) are atomic on most CPUs, and this can only occur if the async update is slower than all synchronous updates combined, but even then, this is an issue and should be handled differently.

Here are are few options that could alleviate the problem:

* At the end of the cm update call, wait for the asynchronous controller to finish. As locking is out of the question, busy waiting could also be tried. However, this could potentially defeat the purpose of using an asynchronous controller.

* Handles should work with atomic primitives, if they are added in the future. The set_value() and get_value() functions could be relaxed stores and loads of the value. This ensures that we're working on tear-free variables without relying on platform specific implementations, with minimal overhead compared to a normal, non-atomic variable.

* There could be a mechanism to check which interfaces are used by which controllers and components. This could let us know if the async update is finished or not, and simply skip the following write, or address it in another manner.

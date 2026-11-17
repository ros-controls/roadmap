Here I describe my thoughts on how we have to clarify the threading model in ros2 control, and test for it, as it causes a lot of headaches for us so far.

I'm working on this a bit already, primarily to solve the issue of lifecycle transitions of async worker blocking update thread

# Hardware component lifecycle: the threading model is not defined

## Terms

- **Worker**: the async thread that calls `read()` and `write()`.
- **Update thread**: the real-time thread of the `controller_manager`.
- **Non-RT thread**: any thread without real-time requirements. For example, the `controller_manager` callback thread, or the worker itself while it skips `read()` and `write()`.
- **Blocking `read()`**: a `read()` that waits for the next hardware packet. If the hardware is silent, it waits without a limit.

## Summary

When we write drivers with ros2_control, the lifecycle of a hardware component has no defined threading model. This issue describes four problems, from the most general to the most specific:

1. **For all hardware components.** `on_error()` and `on_deactivate()` run in the real-time update thread when `read()` or `write()` triggers them. They block all other components and all controllers. The same callbacks run in a non-RT thread when the CLI triggers them. None of this is documented.
   - This forces the author of a hardware interface to write a "fast" and a "slow" version of these callbacks, depending on the trigger.
2. **Async hardware components.** The async worker runs only `read()` and `write()`. The lifecycle transitions run in other threads. Nowhere is it defined how the worker and the transitions interact.
3. **`SELF_PACED` async components.** `read()` blocks most of the time. This makes the problems of point 2 visible as hangs, lost results and an extra `write()`. `DETACHED` has the same edge cases, but we do not know if anyone uses it.
4. **Documentation.** Some behavior is written nowhere. We found it only by reading the code.

## Why we care

We run `ros2_control` on a real-time PC with a watchdog. A deactivation that lasts too long, or a communication cleanup after `on_error()`, trips our watchdog. These callbacks run in the update thread.

We also see a problem when we shut down the process on the IPC. It crashes our whole watchdog process. We do not know the exact cause yet. It has something to do with [this comment on realtime_tools PR#473](https://github.com/ros-controls/realtime_tools/pull/473#discussion_r4042519260). Because `read()` mostly blocks, `stop_thread()` is not propagated correctly, and the ros2_control process does not exit cleanly.

## The Goal

Define the lifecycle expectations and the threading model for normal and async hardware components. Write tests that confirm the implementation follows them. Document them for hardware authors, and protect authors from these traps.

After that, we can review and fix the `SELF_PACED` edge cases.

## Problem 1: `on_error()` and `on_deactivate()` run in the update thread

What happens:

1. The update thread calls `read()` and `write()` of all components, one after another.
2. If `read()` or `write()` returns `ERROR`, the update thread calls `on_error()` of that component.
3. If `write()` returns `DEACTIVATE`, the update thread calls `on_deactivate()` of that component.
4. The update thread waits until the callback returns.
5. During this time, the update thread does not run the other components and the controllers.

The CLI or a service can also start these callbacks. Then the non-RT thread runs them. The update thread does not wait.

| Trigger | Callback | Thread |
|---------|----------|--------|
| `read()` / `write()` returns `ERROR` | `on_error()` | update thread |
| `write()` returns `DEACTIVATE` | `on_deactivate()` | update thread |
| CLI / service | `on_deactivate()`, `on_error()` (after a failed transition) | non-RT thread |

Async components have the same behavior:

1. The worker returns the result.
2. The update thread reads the result in its next cycle.
3. The update thread runs the callback.

The documentation does not state:

- which thread runs these callbacks,
- whether these callbacks must be real-time safe.

### Why this is a problem

- One `on_deactivate()` can run in two different threads. The trigger selects the thread. The author does not know the trigger.
- We cannot ask authors for two versions of each callback:
  - a fast version for the update thread,
  - a slow version for the non-RT thread.
- These callbacks often need time. Examples:
  - set safety flags on the device,
  - wait for an acknowledgment from the device,
  - close the communication.
- In the update thread, a slow callback stops all other components and all controllers.
- A watchdog trips in the update thread.

## Problem 2: the worker runs only `read()` and `write()`

An async component has a worker:

- The worker calls `read()` and `write()`.
- Other threads run all lifecycle callbacks:
  - the update thread (problem 1),
  - the non-RT thread (CLI, services),
  - the rclcpp pre-shutdown callback (shutdown).

Each transition has this sequence:

1. The framework pauses the worker.
2. The pause waits until the current `read()` or `write()` returns.
3. The callback runs.
4. `trigger_read()` resumes the worker.

No document defines how the worker and the callbacks interact. Open questions:

- After `ERROR` or `DEACTIVATE`, which part stops the hardware cycle: the worker or the `controller_manager`?
- Is a result from `read()` / `write()` an event or a state?
  - Event: the framework must not lose it.
  - State: the last value is sufficient.
- Can `read()` continue during a transition?
- Must the worker run all callbacks of its component? This is open for debate.
  - Advantage: one thread for each component. No callback runs at the same time as `read()` or `write()`.
  - Disadvantage: the threading changes for all authors of async components.

## Problem 3: `SELF_PACED` edge cases

The `SELF_PACED` policy:

- [realtime_tools#473](https://github.com/ros-controls/realtime_tools/pull/473) adds the `SELF_PACED` policy.
- In this mode, the worker does not sleep.
- `read()` blocks until the next packet arrives. The hardware sets the cycle time.
- In normal operation, the worker is in a blocking `read()` most of the time.

The pause before each transition waits for this `read()`. After a result, the worker also starts the next `read()` at once. This causes the problems below.

### Watchdog breaks

1. `read()` or `write()` of the worker returns `ERROR` or `DEACTIVATE`.
2. The update thread starts the transition.
3. The update thread waits for the blocking `read()`.
4. During this time, the update thread does not run the other components and the controllers.
5. A hardware watchdog trips.

### Lost result

1. `read()` or `write()` returns `ERROR` or `DEACTIVATE`. The worker stores the result.
2. The worker starts the next `read()` or `write()` at once.
3. The hardware is fast. The next call returns `OK`.
4. The `OK` replaces the stored result.
5. The update thread reads `OK`. The transition never starts.

### One more `write()` after `ERROR` or `DEACTIVATE`

1. `write()` returns `ERROR` or `DEACTIVATE`.
2. The worker starts `read()`. The hardware is healthy, so `read()` returns after one hardware cycle.
3. The component is still `ACTIVE`. The worker calls `write()` again.
4. The hardware gets one more command after the error.

### Shutdown while `read()` blocks

The hardware stops sending packets. For example, the robot is switched off.

1. The shutdown pauses the worker before `on_shutdown()`.
2. The pause waits for the blocked `read()`. `on_shutdown()` does not run.
3. The destructor calls `stop_thread()`. It also waits for the blocked `read()`. The process cannot exit.
4. `shutdown_components()` shuts down the components one after another. The components after the blocked one get no `on_shutdown()`.

The update thread does not wait during the shutdown. It skips a component that is locked.

### Crashes and hangs

- `on_error()` or `on_shutdown()` can leave the worker in an undefined state.
- Before this is signaled to the update thread for callback processing, the worker loop can still run, or block, while the framework shuts down or destroys the component.
- This can stop the shutdown or crash the process.

`DETACHED` looks like it has the same edge cases, but we do not know if anyone uses it.

### Scenarios

| Scenario | Hardware | Problem |
|----------|----------|---------|
| `read()` returns `ERROR` | fast | `ERROR` is lost |
| `read()` returns `ERROR` | silent after the error | `read()` runs again, the pause hangs |
| `write()` returns `ERROR` | healthy | one more `write()` |
| `write()` returns `ERROR` | silent after the error | the pause hangs |
| `write()` returns `ERROR` | fast | `ERROR` is lost |
| `write()` returns `DEACTIVATE` | healthy, silent, fast | same three problems as `ERROR` from `write()` |
| shutdown: `shutdown()` | silent | the pause hangs, `on_shutdown()` does not run |
| shutdown: destructor | silent | `stop_thread()` hangs |
| shutdown: two components | the first is silent | the second component gets no `on_shutdown()` |

## Problem 4: documentation gaps

We found these behaviors only by reading the code.

- During `on_activate()` of a normal or an async hardware component, `read()` and `write()` are skipped. In `debugging.rst` we have:

  > The `update/on_activate/on_deactivate` method of a controller and the `read/write/on_activate/perform_command_mode_switch` methods of a hardware component all run in the context of the realtime update loop.

  This is half-true. For hardware components, `on_activate()` runs in a non-RT thread, and `perform_command_mode_switch()` runs in the RT thread.

  Also, if the user wants to get states during `on_activate()`, they must call `read()` inside `on_activate()`. This is not documented.
- After `on_configure()`, `read()` runs and `write()` is skipped. In `lifecycle_of_a_hardware_component.rst`, a note says the opposite:

  > But currently, all command interfaces are available and will be written, see this issue describing the situation.

  The same section also says that the hardware component implementation can "continue using the command received from the `CommandInterfaces` or skip them completely". The framework already skips `write()`. A component can only use the commands in `read()` (for example with `get_command()`).

## Next steps

1. Agree on the threading model for normal hardware components (problem 1).
2. Agree on the threading model for async hardware components (problem 2).
3. Write tests that confirm both models. Write one test for each row of the scenario table in problem 3.
4. Document the agreed model for hardware authors (problem 4).
5. Fix the `SELF_PACED` edge cases.

## Discussion: our suggestion for expected behavior

Rules 1, 2 and 8 apply to all hardware components. Rules 3 and 4 already hold for normal components, because the recursive `component_mutex_` of the component guards each transition, `read()` and `write()`. The other rules apply to async components.

1. **The update thread never waits for a hardware component.** No transition and no worker blocks the update thread. Its cycle time stays bounded.
2. **`on_error()` and `on_deactivate()` always run in a non-RT thread.** This is the `controller_manager` non-RT thread, or the worker itself with `read()` and `write()` skipped. The same rule applies for all triggers: a result from `read()` / `write()`, the CLI, or a service. The author then has time to send stop flags or to close the communication.
3. **A lifecycle callback never runs at the same time as `read()` or `write()` of the same component.**
4. **Lifecycle callbacks of one component never run at the same time.** For example, `on_shutdown()` waits until a running `on_error()` or `on_deactivate()` is finished.
5. **`ERROR` and `DEACTIVATE` are events.** The framework never loses them. A later `OK` does not replace them.
6. **After the worker gets `ERROR` it starts no new `read()` or `write()`. After `DEACTIVATE`, it starts no new `write()`.**
7. **Shutdown always finishes.** `on_shutdown()` runs once for each component. After `on_shutdown()`, the worker is stopped and joined. One component cannot prevent the shutdown of another one. The process can always exit cleanly.
8. **The documentation states three facts for each lifecycle callback:**
   - the thread that runs it,
   - whether it must be real-time safe,
   - what the worker does at that time.

### Expected behavior per transition for a `SELF_PACED` hardware interface

#### `on_error()`

| Trigger | Worker state when triggered | Expected behavior |
|---------|-----------------------------|-------------------|
| `read()` returns `ERROR` | `read()` just returned | No `write()` in this cycle. No new `read()` or `write()`. `on_error()` runs once, in a non-RT thread. The update thread continues with the other components. |
| `write()` returns `ERROR` | `write()` just returned | No new `read()` or `write()`. `on_error()` runs once, in a non-RT thread. The update thread continues. |
| a transition callback returns `ERROR` | worker paused | `on_error()` runs once, in the same non-RT thread. |

After `on_error()`:

- `SUCCESS`: the state is `UNCONFIGURED`. The worker is paused. It runs no `read()` or `write()` until `on_configure()` runs again.
- `FAILURE` / `ERROR`: the state is `FINALIZED`. The worker is stopped and joined.

#### `on_deactivate()`

| Trigger | Worker state when triggered | Expected behavior |
|---------|-----------------------------|-------------------|
| `write()` returns `DEACTIVATE` | `write()` just returned | No new `write()`. `on_deactivate()` runs once, in a non-RT thread. The update thread continues. |
| CLI / service request | in a blocking `read()` | No new `write()` after the request. The update thread is not blocked. Open: does `on_deactivate()` wait for the `read()` (see the open questions)? |
| CLI / service request | in `write()` | `write()` finishes. Then `on_deactivate()` runs. No new `write()`. |
| CLI / service request | between two cycles | `on_deactivate()` runs. No new `write()`. |

After `on_deactivate()` returns `SUCCESS`:

- The state is `INACTIVE`.
- The worker runs `read()` only, and never `write()`.

#### `on_shutdown()`

| Shutdown request while | Expected behavior |
|------------------------|-------------------|
| the worker is in a blocking `read()`, hardware healthy | `on_shutdown()` runs after `read()` returns (at most one hardware cycle). Then the worker is stopped. |
| the worker is in a blocking `read()`, hardware silent | `on_shutdown()` runs. Then the worker is stopped. Open: how long does the shutdown wait (see the open questions)? |
| the worker is in `write()` | `write()` finishes. Then `on_shutdown()` runs. Then the worker is stopped. |
| the worker is between two cycles | No new `read()` or `write()` starts. `on_shutdown()` runs. Then the worker is stopped. |
| `on_error()` runs | `on_shutdown()` waits until `on_error()` is finished. Then it runs once. |
| `on_deactivate()` runs | `on_shutdown()` waits until `on_deactivate()` is finished. Then it runs once. |
| the worker is paused (`UNCONFIGURED`) | `on_shutdown()` runs. Then the worker is stopped. |

For all rows:

- Every other component also gets `on_shutdown()`, even if one component hangs.
- The component destructor never hangs and never crashes.

### Open questions

- May `read()` continue while `on_deactivate()` runs, or must the worker skip both `read()` and `write()`?
- `ERROR` / `DEACTIVATE` from `read()` / `write()`: should the worker itself run `on_error()` / `on_deactivate()` (with `read()` and `write()` skipped), or should it hand them to the `controller_manager` non-RT thread?
- Who tells the update thread that the component is no longer available, and when?
- What is the time limit for a transition, and what happens when a transition takes longer?
- How exactly do we shut down async components after the `on_shutdown` signal?
  - The worker can block in `read()` before the shutdown pauses it. Then the shutdown loop stops at this component. The loop runs in the pre-shutdown callback, one component after another.
  - The other components get no `on_shutdown()`, and the process cannot exit.
  - The update thread is not blocked, because it skips a locked component.
  - How do we signal a forceful stop to the async worker during shutdown? See the [relevant PR#473 comment](https://github.com/ros-controls/realtime_tools/pull/473#discussion_r4042519260).
- Smaller point: the framework runs `on_deactivate()` and then `on_shutdown()` for an `ACTIVE` component. It runs only `on_shutdown()` for an `INACTIVE` or `UNCONFIGURED` component. It never runs `on_cleanup()`.
  - The author must close the communication in `on_shutdown()`, and also in `on_cleanup()` and `on_error()`. Is this intended?
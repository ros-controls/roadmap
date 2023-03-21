# Design proposals for controllers

When writing controllers - there is no consistent naming convention we are using for different variables and this is quite confusing, especially, when debugging things.

Proposal is to have the following names for the 4 important values in controllers:

- **Reference** - input to controller from a topic or preceding controller.
- **Feedback** - measured state of the system controller controls.
- **Error** - control error (*Reference* - *Feedback*)
- **Output** - controller's output, i.e., values written to command interfaces. This are either values of references for the following controller or input to the controlled system.

This is summarized in the following figure:

![Control loop with names](images/control_loop_naming.png)

Beside this, every controller **must have** `~/controller_state` topic where at least the four values mentioned above have to be output (of course if applicable).

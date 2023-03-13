# Design proposals for controllers

When wrting controllers - the is no consistent naming convntion we are using for differnet variables and this is quite confusing, especially, when debugging things.

Proposal is to have the following names for the 4 important values in controllers:

- **Reference** - input to controller from a topic or preceeding controller.
- **Feedback** - measured state of the system controller controls.
- **Error** - control error (*Reference* - *Feedback*)
- **Output** - controlers output, i.e., values written to command interfaces. This are either values of references for the following controller or input to the controlled system.

This is summarized in the following figure:

![Control loop with names](images/control_loop_naming.png)

Beside this issue, use of work "state" is not really suitable for outputing controller's debug information.
Proposal is to rename this topic to "status" which every controller then **must have** outputing at least the four values mentioned above (if applicable).

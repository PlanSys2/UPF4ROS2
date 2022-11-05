## Representation of an action instance that appears in a plan.

# Optional. A unique identifier of the action that might be used to refer to it (e.g. in HTN plans).
string id
# name of the action
string action_name
# Parameters of the action instance, required to be constants.
upf_msgs/Atom[] parameters

bool time_triggered

# Start time of the action. The default 0 value is OK in the case of non-temporal planning
# feature: [DURATIVE_ACTIONS]
upf_msgs/Real start_time
# End time of the action. The default 0 value is OK in the case of non-temporal planning
# feature: [DURATIVE_ACTIONS]
upf_msgs/Real end_time

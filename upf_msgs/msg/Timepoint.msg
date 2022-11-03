## Symbolic reference to an absolute time.
## It might represent:
## - the time of the initial/final state, or
## - the start/end of the containing action.
##
## It is currently composed of a single field whose interpretation might be context dependent
## (e.g. "START" refers to the start of the containing action).
##
## In the future, it could be extended to refer, e.g., to the start of a particular action/subtask
## by adding an additional field with the identifier of an action/subtask.

# Global start of the planning problem. This is context independent and represents the time at which the initial state holds.
uint8 GLOBAL_START=0
# Global end of the planning problem. This is context independent and represents the time at which the final state holds.
uint8 GLOBAL_END=1
# Start of the container (typically the action or method) in which this symbol occurs
uint8 START=2
# End of the container (typically the action or method) in which this symbol occurs
uint8 END=3

uint8 kind

# If non-empty, identifies the container of which we are extracting the start/end timepoint.
# In the context of a task-network or of a method, this could be the `id` of one of the subtasks.
# feature: hierarchies
string container_id

## Representation of an abstract or primitive task that should be achieved,
## required either in the initial task network or as a subtask of a method.
##
## Example:  task of sending a `robot` to the KITCHEN
##   - t1: goto(robot, KITCHEN)

# Identifier of the task, required to be unique in the method/task-network where the task appears.
# The `id` is notably used to refer to the start/end of the task.
#
# Example: t1
string id

# Name of the task that should be achieved. It might either
#  - an abstract task if the name is the one of a task declared in the problem
#  - a primitive task if the name is the one of an action declared in the problem
#
# Example:
#  - "goto" (abstract task)
#  - "move" (action / primitive task)
string task_name

# Example: (for a "goto" task)
#  - robot    (a parameter from an outer scope)
#  - KITCHEN  (a constant symbol in the problem)
upf_msgs/Expression[] parameters

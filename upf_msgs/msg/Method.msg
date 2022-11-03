# A method describes one possible way of achieving a task.
#
# Example: A method that make a "move" action and recursively calls itself until reaching the destination.

# A name that uniquely identify the method.
# This is mostly used for user facing output or plan validation.
#
# Example: "m-recursive-goto"
string name

# Example: [robot: Robot, source: Location, intermediate: Location, destination: Location]
Parameter[] parameters

# The task that is achieved by the method.
# A subset of the parameters of the method will typically be used to
# define the task that is achieved.
#
# Example: goto(robot, destination)
upf_msgs/Task achieved_task

# A set of subtasks that should be achieved to carry out the method.
# Note that the order of subtasks is irrelevant and that any ordering constraint should be
# specified in the `constraints` field.
#
# Example:
#  - t1: (move robot source intermediate)
#  - t2: goto(robot destination)
upf_msgs/Task[] subtasks

# Constraints enable the definition of ordering constraints as well as constraints
# on the allowed instantiation of the method's parameters.
#
# Example:
#  - end(t1) < start(t2)
#  - source != intermediate
upf_msgs/Expression[] constraints

# Conjunction of conditions that must hold for the method to be applicable.
# As for the conditions of actions, these can be temporally qualified to refer to intermediate timepoints.
# In addition to the start/end of the method, the temporal qualification might refer to the start/end of
# one of the subtasks using its identifier.
#
# Example:
#  - [start] loc(robot) == source
#  - [end(t1)] loc(robot) == intermediate
#  - [end] loc(robot) == destination
upf_msgs/Condition[] conditions

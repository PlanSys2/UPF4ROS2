## A task network defines a set of subtasks and associated constraints.
## It is intended to be used to define the initial task network of the hierarchical problem.
##
## Example: an arbitrary robot should go to the KITCHEN before time 100

# robot: Location
upf_msgs/Parameter[] variables
# t1: goto(robot, KITCHEN)
upf_msgs/Task[] subtasks
# end(t1) <= 100
upf_msgs/Expression[] constraints

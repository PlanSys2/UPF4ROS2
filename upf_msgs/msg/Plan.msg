# An ordered sequence of actions that appear in the plan.
# The order of the actions in the list must be compatible with the partial order of the start times.
# In case of non-temporal planning, this allows having all start time at 0 and only rely on the order in this sequence.
uint8 kind
upf_msgs/ActionInstance[] actions

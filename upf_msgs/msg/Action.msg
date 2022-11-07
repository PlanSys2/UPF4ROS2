## Unified action representation that represents any kind of actions.

# Action name. E.g. "move"
string name

# Typed and named parameters of the action.
upf_msgs/Parameter[] parameters

# If set, the action is durative. Otherwise it is instantaneous.
# features: DURATIVE_ACTIONS
upf_msgs/Duration[<=1] duration

# Conjunction of conditions that must hold for the action to be applicable.
upf_msgs/Condition[] conditions

# Conjunction of effects as a result of applying this action.
upf_msgs/Effect[] effects

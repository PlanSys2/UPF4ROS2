## A state-dependent variable.

string name
# Return type of the fluent.
string value_type "up:bool"
# Typed and named parameters of the fluent.
upf_msgs/Parameter[] parameters

# If non-empty, then any state variable using this fluent that is not explicitly given a value in the initial state
# will be assumed to have this default value.
# This allows mimicking the closed world assumption by setting a "false" default value to predicates.
# Note that in the initial state of the problem message, it is assumed that all default values are set.
upf_msgs/Expression[<=1] default_value

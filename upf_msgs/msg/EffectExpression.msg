## An effect expression is of the form `FLUENT OP VALUE`.
## We explicitly restrict the different types of effects by setting the allowed operators.

# The `fluent` is set to the corresponding `value`
uint8 ASSIGN=0
# The `fluent` is increased by the amount `value`
# features: INCREASE_EFFECTS
uint8 INCREASE=1
# The `fluent` is decreased by the amount `value`
# features: DECREASE_EFFECTS
uint8 DECREASE=2

uint8 kind

# Expression that must be of the STATE_VARIABLE kind.
upf_msgs/Expression fluent
upf_msgs/Expression value
upf_msgs/Variable[] forall

# Optional. If the effect is conditional, then the following field must be set.
# In this case, the `effect` will only be applied if the `condition`` holds.
# If the effect is unconditional, the effect is set to the constant 'true' value.
# features: CONDITIONAL_EFFECT
upf_msgs/Expression condition

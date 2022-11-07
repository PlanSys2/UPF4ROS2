## Representation of an effect that allows qualifying the effect expression, e.g., to make it a conditional effect.

# Required. The actual effect that should take place.
upf_msgs/EffectExpression effect

# Optional. If the effect is within a durative action, the following must be set and will specify when the effect takes place.
# features: DURATIVE_ACTIONS
upf_msgs/Timing[<=1] occurrence_time

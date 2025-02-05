## Represents an effect that will occur sometime beyond the initial state. (similar to timed initial literals)

# Required. An effect expression that will take place sometime in the future (i.e. not at the intial state) as specified by the temporal qualifiation.
upf_msgs/EffectExpression[] effect
# Required. Temporal qualification denoting when the timed fact will occur.
upf_msgs/Timing occurrence_time

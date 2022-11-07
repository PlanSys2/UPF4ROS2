## A Goal is currently an expression that must hold either:
## - in the final state,
## - over a specific temporal interval (under the `timed_goals` features)

# Goal expression that must hold in the final state.
upf_msgs/Expression goal

# Optional. If specified the goal should hold over the specified temporal interval (instead of on the final state).
# features: TIMED_GOALS
upf_msgs/TimeInterval[<=1] timing

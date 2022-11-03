## Message sent by engine.
## Contains the engine exit status as well as the best plan found if any.


# ==== Engine stopped normally ======

# Valid plan found
# The `plan` field must be set.
uint8 SOLVED_SATISFICING=0
# Plan found with optimality guarantee
# The `plan` field must be set and contains an optimal solution.
uint8 SOLVED_OPTIMALLY=1
# No plan exists
uint8 UNSOLVABLE_PROVEN=2
# The engine was not able to find a solution but does not give any guarantee that none exist
# (i.e. the engine might not be complete)
uint8 UNSOLVABLE_INCOMPLETELY=3

# ====== Engine exited before making any conclusion ====
# Search stopped before concluding SOLVED_OPTIMALLY or UNSOLVABLE_PROVEN
# If a plan was found, it might be reported in the `plan` field

# The engine ran out of time
uint8 TIMEOUT=13
# The engine ran out of memory
uint8 MEMOUT=14
# The engine faced an internal error.
uint8 INTERNAL_ERROR=15
# The problem submitted is not supported by the engine.
uint8 UNSUPPORTED_PROBLEM=16

# ====== Intermediate answer ======
# This Answer is an Intermediate Answer and not a Final one
uint8 INTERMEDIATE=17

uint8 status

# Optional. Best plan found if any.
upf_msgs/Plan plan

# A set of engine specific values that can be reported, for instance
# - "grounding-time": "10ms"
# - "expanded-states": "1290"
string[] metric_names
string[] metric_values

# Optional log messages about the engine's activity.
# Note that it should not be expected that logging messages are visible to the end user.
# If used in conjunction with INTERNAL_ERROR or UNSUPPORTED_PROBLEM, it would be expected to have at least one log message at the ERROR level.
upf_msgs/LogMessage[] log_messages

# Synthetic description of the engine that generated this message.
string engine_name

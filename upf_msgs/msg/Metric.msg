
# Minimize the action costs expressed in the `action_costs` field
uint8 MINIMIZE_ACTION_COSTS=0

# Minimize the length of the resulting sequential plan
uint8 MINIMIZE_SEQUENTIAL_PLAN_LENGTH=1

# Minimize the makespan in case of temporal planning
# features: durative_actions
uint8 MINIMIZE_MAKESPAN=2

# Minimize the value of the expression defined in the `expression` field
uint8 MINIMIZE_EXPRESSION_ON_FINAL_STATE=3

# Maximize the value of the expression defined in the `expression` field
uint8 MAXIMIZE_EXPRESSION_ON_FINAL_STATE=4

# Maximize the number of goals reached, weighted by cost
uint8 OVERSUBSCRIPTION=5
    
uint8 kind


# Expression to minimize/maximize in the final state.
# Empty, if the `kind` is not {MIN/MAX}IMIZE_EXPRESSION_ON_FINAL_STATE
Expression expression

# If `kind == MINIMIZE_ACTION_COSTS``, then each action is associated to a cost expression.
#
# TODO: Document what is allowed in the expression. See issue #134
# In particular, for this metric to be useful in many practical problems, the cost expression
# should allow referring to the action parameters (and possibly the current state at the action start/end).
# This is very awkward to do in this setting where the expression is detached from its scope.
upf_msgs/Action[] action_cost_names
upf_msgs/Expression[] action_cost_expr

upf_msgs/Expression[<=1] default_action_cost

# List of goals used to define the oversubscription planning problem.
# Empty, if the `kind` is not OVERSUBSCRIPTION
upf_msgs/GoalWithCost[] goals
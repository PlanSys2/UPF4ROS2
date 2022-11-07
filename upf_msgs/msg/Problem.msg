string domain_name
string problem_name
upf_msgs/TypeDeclaration[] types
upf_msgs/Fluent[] fluents
upf_msgs/ObjectDeclaration[] objects

# List of actions in the domain.
upf_msgs/Action[] actions

# Initial state, including default values of state variables.
upf_msgs/Assignment[] initial_state

# Facts and effects that are expected to occur strictly later than the initial state.
# features: TIMED_EFFECT
upf_msgs/TimedEffect[] timed_effects

# Goals of the planning problem.
upf_msgs/Goal[] goals

# The plan quality metrics
upf_msgs/Metric[] metrics


# If the problem is hierarchical, defines the tasks and methods as well as the initial task network.
# features: hierarchical
upf_msgs/Hierarchy[<=1] hierarchy

# all features of the problem
uint8[] features

## Features of the problem.
## Features are essential in that not supporting a feature `X` should allow disregarding any field tagged with `features: [X]`.

# PROBLEM_CLASS
uint8 ACTION_BASED=0
uint8 HIERARCHICAL=26
# PROBLEM_TYPE
uint8 SIMPLE_NUMERIC_PLANNING=30
uint8 GENERAL_NUMERIC_PLANNING=31
# TIME
uint8 CONTINUOUS_TIME=1
uint8 DISCRETE_TIME=2
uint8 INTERMEDIATE_CONDITIONS_AND_EFFECTS=3
uint8 TIMED_EFFECT=4
uint8 TIMED_GOALS=5
uint8 DURATION_INEQUALITIES=6
# EXPRESSION_DURATION
uint8 STATIC_FLUENTS_IN_DURATION=27
uint8 FLUENTS_IN_DURATION=28
# NUMBERS
uint8 CONTINUOUS_NUMBERS=7
uint8 DISCRETE_NUMBERS=8
# CONDITIONS_KIND
uint8 NEGATIVE_CONDITIONS=9
uint8 DISJUNCTIVE_CONDITIONS=10
uint8 EQUALITY=11
uint8 EXISTENTIAL_CONDITIONS=12
uint8 UNIVERSAL_CONDITIONS=13
# EFFECTS_KIND
uint8 CONDITIONAL_EFFECTS=14
uint8 INCREASE_EFFECTS=15
uint8 DECREASE_EFFECTS=16
# TYPING
uint8 FLAT_TYPING=17
uint8 HIERARCHICAL_TYPING=18
# FLUENTS_TYPE
uint8 NUMERIC_FLUENTS=19
uint8 OBJECT_FLUENTS=20
# QUALITY_METRICS
uint8 ACTIONS_COST=21
uint8 FINAL_VALUE=22
uint8 MAKESPAN=23
uint8 PLAN_LENGTH=24
uint8 OVERSUBSCRIPTION=29
# SIMULATED_ENTITIES
uint8 SIMULATED_EFFECTS=25

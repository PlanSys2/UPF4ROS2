## The kind of an expression, which gives information related to its structure.

# If non-empty, the expression is a single atom.
# For instance `3`, `+`, `kitchen`, `at-robot`, ...
upf_msgs/Atom[<=1] atom
# If the `atom` field is empty, then the expression is a list of sub-expressions,
# typically representing the application of some arguments to a function or fluent.
# For instance `(+ 1 3)`, (at-robot l1)`, `(>= (battery_level) 20)`

# Type of the expression. For instance "int", "location", ...
string type

# Kind of the expression, specifying the content of the expression.
# This is intended to facilitate parsing of the expression.
    
# Default value, should not be used. Drop it if we are sure to never need it.
uint8 UNKNOWN=0

# Constant atom. For instance `3` or `kitchen` (where `kitchen` is an object defined in the problem)
uint8 CONSTANT=1

# Atom symbol representing a parameter from an outer scope. For instance `from` that would appear inside a `(move from to - location)` action.
uint8 PARAMETER=2

# Atom symbol representing a variable from an outer scope.
# This is typically used to represent the variables that are existentially or universally qualified in expressions.
uint8 VARIABLE=7

# Atom symbol representing a fluent of the problem. For instance `at-robot`.
uint8 FLUENT_SYMBOL=3

# Atom representing a function. For instance `+`, `=`, `and`, ...
uint8 FUNCTION_SYMBOL=4

# List. Application of some parameters to a fluent symbol. For instance `(at-robot l1)` or `(battery-charged)`
# The first element of the list must be a FLUENT_SYMBOL
uint8 STATE_VARIABLE=5

# List. The expression is the application of some parameters to a function. For instance `(+ 1 3)`.
# The first element of the list must be a FUNCTION_SYMBOL
uint8 FUNCTION_APPLICATION=6

# Atom symbol. Unique identifier of a task or action in the current scope.
uint8 CONTAINER_ID=8

uint8 kind

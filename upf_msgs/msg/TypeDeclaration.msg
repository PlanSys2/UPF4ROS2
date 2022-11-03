## Declares the existence of a symbolic type.

# Name of the type that is declared.
string type_name

# Optional. If the string is non-empty, this is the parent type of `type_name`.
# If set, the parent type must have been previously declared (i.e. should appear earlier in the problem's type declarations.
# feature: HIERARCHICAL_TYPING
string parent_type
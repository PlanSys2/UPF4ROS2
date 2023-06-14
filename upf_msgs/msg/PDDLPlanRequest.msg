# Problem that should be solved.

# Specify if the domain/model string contains a file path, or the content
uint8 FILE=0
uint8 RAW=1
uint8 mode

string domain
string problem
string problem_name

uint8 SATISFIABLE=0
uint8 SOLVED_OPTIMALLY=1
uint8 resolution_mode

# Max allowed runtime time in seconds.
float64 timeout

# Engine specific options to be passed to the engine
string engine_option_names
string engine_option_values

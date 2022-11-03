## A freely formatted logging message.
## Each message is annotated with its criticality level from the minimal (DEBUG) to the maximal (ERROR).
## Criticality level is expected to be used by an end user to decide the level of verbosity.

uint8 DEBUG=0
uint8 INFO=1
uint8 WARNING=2
uint8 ERROR=3

uint8 level
string message
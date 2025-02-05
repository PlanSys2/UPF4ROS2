## Message sent by the validator.

# The Plan is valid for the Problem.
uint8 VALID=0
# The Plan is not valid for the Problem.
uint8 INVALID=0
uint8 status

# Optional. Information given by the engine to the user.
upf_msgs/LogMessage[] log_messages

# Synthetic description of the engine that generated this message.
string engine

ValidationMetric[] metrics


## A contiguous slice of time represented as an interval `[lower, upper]` where `lower` and `upper` are time references.
## The `is_left_open` and `is_right_open` fields indicate whether the interval is
## opened on left and right side respectively.

bool is_left_open
upf_msgs/Timing lower
bool is_right_open
upf_msgs/Timing upper

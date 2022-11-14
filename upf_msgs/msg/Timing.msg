## Represents a time (`timepoint` + `delay`), that is a time defined relatively to a particular `timepoint`.
## Note that an absolute time can be defined by setting the `delay` relative to the `GLOBAL_START`` which is the reference time.

upf_msgs/Timepoint timepoint
upf_msgs/Real[<=1] delay


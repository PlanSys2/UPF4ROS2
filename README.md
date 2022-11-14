# UPF4ROS2
<!---[![GitHub Action
Status](https://github.com/PlanSys2/UPF4ROS2/workflows/main/badge.svg)](https://github.com/PlanSys2/UPF4ROS2)
-->

This repository contains a UPF TSB for ROS 2

## Install and building

```
$ pip install --pre unified-planning[pyperplan,tamer]
$ cd <upf_workspace>
$ cd src
$ git clone https://github.com/PlanSys2/UPF4ROS2.git
$ vcs import . < UPF4ROS2/upf.repos
$ cd ..
$ colcon build --symlink-install
```

## Usage

`$ ros2 launch  upf4ros2 upf4ros2.launch.py`

## Nodes

* **upf4ros**
  * Services:
    * `/upf4ros2/add_action` `[upf_msgs/srv/AddAction]` 
    * `/upf4ros2/add_fluent` `[upf_msgs/srv/AddFluent]` 
    * `/upf4ros2/add_goal` `[upf_msgs/srv/AddGoal]` 
    * `/upf4ros2/add_object` `[upf_msgs/srv/AddObject]` 
    * `/upf4ros2/new_problem` [upf_msgs/srv/NewProblem]` 
    * `/upf4ros2/set_initial_value` [upf_msgs/srv/SetInitialValue]` 
    * `/upf4ros2/set_problem` [upf_msgs/srv/SetProblem]`
  * Actions:
    * `/upf4ros2/planOneShotPDDL` `[upf_msgs/action/PDDLPlanOneShot]` 
    * `/upf4ros2/planOneShot` `[upf_msgs/action/PlanOneShot]` 
    * `/upf4ros2/planOneShotRemote` `[upf_msgs/action/PlanOneShotRemote]` 

## Acknowledgments

<img src="https://www.aiplan4eu-project.eu/wp-content/uploads/2021/07/euflag.png" width="60" height="40">

This library is being developed for the AIPlan4EU H2020 project (https://aiplan4eu-project.eu) that is funded by the European Commission under grant agreement number 101016442.

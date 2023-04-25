# UPF4ROS2

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

See [main repository](https://github.com/PlanSys2/UPF4ROS2) for further instructions.

## Demo
### [Demo 1](https://www.youtube.com/watch?v=fObz6H1DnXs)
This demo consists of creating the problem from a ros2 node to navigate from living room to the entrance.

`$ ros2 launch upf4ros2 upf4ros2.launch.py`

`$ ros2 launch upf4ros2_demo upf4ros2_demo1.launch.py`

### Demo 1 (pddl file)
This demo consists of creating the problem from a pddl domain and problem file.

`$ ros2 launch upf4ros2 upf4ros2.launch.py`

`$ ros2 launch upf4ros2_demo upf4ros2_demo1_pddlfile.launch.py`

# UPF4ROS2 for REAP

This repository contains a fork of [UPF4ROS2](https://github.com/PlanSys2/UPF4ROS2) for usage in the [REAP Framework](https://github.com/UniBwM-IFS-AILab/REAP) as described in the [ICAPS 2023 Demo paper](https://icaps23.icaps-conference.org/program/demos/#3216). UPF4ROS2 is a plugin for [PlanSys2](https://plansys2.github.io/) which allows usage of the [Unified Planning Framework](https://github.com/aiplan4eu/unified-planning) within PlanSys2.

## Install and building

```
$ pip install --pre unified-planning[pyperplan,tamer]
$ cd <Plansys2 directory>
$ cd src
$ git clone https://github.com/OvrK12/UPF4ROS2.git
$ vcs import . < UPF4ROS2/upf.repos
$ cd ..
$ colcon build --symlink-install
```
In src/UPF4ROS2/upf4ros2 run `sudo python3 setup.py install`. After every build, you also need to run `source install/setup.bash`
## Demo
In the first terminal run
`$ ros2 launch upf4ros2 upf4ros2.launch.py`

In the second terminal run
`$ ros2 launch upf4ros2_demo traverse_areas.launch.py`


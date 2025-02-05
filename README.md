# UPF4ROS2

[![main](https://github.com/igonzf/UPF4ROS2/actions/workflows/main.yaml/badge.svg)](https://github.com/igonzf/UPF4ROS2/actions/workflows/main.yaml)

This repository integrates the Unified Planning Framework (UPF) into the ROS 2 ecosystem, providing a modular and standardized solution for automated planning in robotic systems. This project is part of the European initiative AIPlan4EU, which aims to develop automated planning tools that are accessible and applicable across different engineering domains.

## Install and building

```
$ pip install --pre unified-planning[pyperplan,tamer]
$ pip install ConfigSpace
$ pip install typing_extensions==4.7.1 --upgrade
$ cd <upf_workspace>
$ cd src
$ git clone https://github.com/PlanSys2/UPF4ROS2.git
$ vcs import . < UPF4ROS2/upf.repos
$ cd ..
$ colcon build --symlink-install
```

### Install UPF from sources

```
$ cd src
$ git clone https://github.com/aiplan4eu/unified-planning.git
$ cd unified-planning
$ python3 -m pip install -r requirements.txt
$ python3 -m pip install -r dev-requirements.txt
$ python3 -m pip install tarski[arithmetic]
$ sudo apt install -y gringo
$ python3 -m pip install black==22.6.0
$ python3 -m black --check --exclude=unified_planning/grpc/generated/ .
$ python3 -m mypy unified_planning
$ python3 scripts/test_imports.py
$ cd ..
$ git clone -b 869e7ab06cf23c5541a47f46209159fd51d8021f https://github.com/aiplan4eu/up-tamer
$ python3 -m pip install up-tamer/
$ git clone -b ac2b04d2d41c20b15f7c4143c19947f9704d1888 https://github.com/aiplan4eu/up-pyperplan
$ python3 -m pip install up-pyperplan/
```

Install Java 17, with:

```
$ sudo apt install openjdk-17-jdk openjdk-17-jre
```

```
$ git clone https://gitlab.com/enricos83/ENHSP-Public.git
$ cd ENHSP-Public; git checkout enhsp20-0.9.5; ./compile; cd ..
$ mkdir .planners; mv ENHSP-Public/enhsp-dist .planners/enhsp-20; rm -rf ENHSP-Public
```

Check with :

```
$ cd unified-planning
$ bash run_tests.sh
```

## Usage

`$ ros2 launch  upf4ros2 upf4ros2.launch.py`

## Nodes

- **upf4ros**
  - Services:
    - `/upf4ros2/add_action` `[upf_msgs/srv/AddAction]`
    - `/upf4ros2/add_fluent` `[upf_msgs/srv/AddFluent]`
    - `/upf4ros2/add_goal` `[upf_msgs/srv/AddGoal]`
    - `/upf4ros2/add_object` `[upf_msgs/srv/AddObject]`
    - `/upf4ros2/new_problem` [upf_msgs/srv/NewProblem]`
    - `/upf4ros2/set_initial_value` [upf_msgs/srv/SetInitialValue]`
    - `/upf4ros2/set_problem` [upf_msgs/srv/SetProblem]`
  - Actions:
    - `/upf4ros2/planOneShotPDDL` `[upf_msgs/action/PDDLPlanOneShot]`
    - `/upf4ros2/planOneShot` `[upf_msgs/action/PlanOneShot]`
    - `/upf4ros2/planOneShotRemote` `[upf_msgs/action/PlanOneShotRemote]`

## Demo

### [Demo 1](https://www.youtube.com/watch?v=fObz6H1DnXs)

This demo consists of creating the problem from a ros2 node to navigate from living room to the entrance.

`$ ros2 launch upf4ros2 upf4ros2.launch.py`

`$ ros2 launch upf4ros2_demo upf4ros2_demo1.launch.py`

### Demo 1 (pddl file)

This demo consists of creating the problem from a pddl domain and problem file.

`$ ros2 launch upf4ros2 upf4ros2.launch.py`

`$ ros2 launch upf4ros2_demo upf4ros2_demo1_pddlfile.launch.py`

### Demo 1 (bash)

This demo consists of creating the problem from the command line. For easier use you can use the script in /upf4ros2_demo/resource/upf_problem.sh

`$ ros2 launch upf4ros2 upf4ros2.launch.py`

`$ ./upf_problem.sh`

`$ ros2 launch upf4ros2_demo upf4ros2_demo1_bash.launch.py`

### [Demo 2](https://www.youtube.com/watch?v=HJ46htSfPZY)

This demo consists of creating the problem from a ROS 2 node to navigate from living room to the entrance.
For run this demo I used the simulated TIAGo robot from this [repo](https://github.com/jmguerreroh/ros2_computer_vision)

`$ ros2 launch upf4ros2 upf4ros2.launch.py`

`$ ros2 launch upf4ros2_demo upf4ros2_demo2.launch.py`

### Demo 3

This demo consists of creating the problem from a ROS 2 node to navigate and check a list of waypoints starting from living room.
For run this demo I used the simulated TIAGo robot from this [repo](https://github.com/jmguerreroh/ros2_computer_vision)

`$ ros2 launch upf4ros2 upf4ros2.launch.py`

`$ ros2 launch upf4ros2_demo upf4ros2_demo3.launch.py`

There are two alternatives:

- Regular case: Illustrated in this [video](https://youtu.be/2nKqxGYlHk8)

- Replanning case: one of the waypoints is not reachable and it is necessary to replan. Illustrated in this [video](https://youtu.be/UJncg7GPCro)

## Acknowledgments

<img src="https://www.aiplan4eu-project.eu/wp-content/uploads/2021/07/euflag.png" width="60" height="40">

This library is being developed for the AIPlan4EU H2020 project (https://aiplan4eu-project.eu) that is funded by the European Commission under grant agreement number 101016442.

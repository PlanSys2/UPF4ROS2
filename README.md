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

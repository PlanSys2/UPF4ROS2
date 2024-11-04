#! /bin/bash
# Script to create a example problem to UPF

# Create new problem

ros2 service call /upf4ros2/srv/new_problem upf_msgs/srv/NewProblem "{problem_name: test}"

# Create the fluent robot_at

ros2 service call /upf4ros2/srv/add_fluent upf_msgs/srv/AddFluent "{problem_name: test, fluent:{name: robot_at, value_type: up:bool, parameters:[{name: l1, type: Location}]}, default_value:{expressions:[{atom:[{symbol_atom:[], int_atom:[], real_atom:[], boolean_atom:[False]}], type: up:bool, kind: 1}], level:[0]}}"

# Create the objects l1 and l2 (Location)

ros2 service call /upf4ros2/srv/add_object upf_msgs/srv/AddObject "{problem_name: test, object:{name: l1, type: Location}}"

ros2 service call /upf4ros2/srv/add_object upf_msgs/srv/AddObject "{problem_name: test, object:{name: l2, type: Location}}"

# Set initial values to the fluent robot_at with the objects l1 and l2: robot_at(l1) and robot_at(l2)
# The robot is at l1 but not at l2

ros2 service call /upf4ros2/srv/set_initial_value upf_msgs/srv/SetInitialValue "{problem_name: test, expression: {expressions:[{atom:[{symbol_atom:['robot_at l1'], int_atom:[], real_atom:[], boolean_atom:[]}], type: up:bool, kind: 5}, {atom:[{symbol_atom:['robot_at'], int_atom:[], real_atom:[], boolean_atom:[]}], type: up:bool, kind: 3}, {atom:[{symbol_atom:['l1'], int_atom:[], real_atom:[], boolean_atom:[]}], type: Location, kind: 1}], level:[0, 1, 1]}, value: {expressions:[{atom:[{symbol_atom:[], int_atom:[], real_atom:[], boolean_atom:[True]}], type: up:bool, kind: 1}], level:[0]}}"

ros2 service call /upf4ros2/srv/set_initial_value upf_msgs/srv/SetInitialValue "{problem_name: test, expression: {expressions:[{atom:[{symbol_atom:['robot_at l2'], int_atom:[], real_atom:[], boolean_atom:[]}], type: up:bool, kind: 5}, {atom:[{symbol_atom:['robot_at'], int_atom:[], real_atom:[], boolean_atom:[]}], type: up:bool, kind: 3}, {atom:[{symbol_atom:['l2'], int_atom:[], real_atom:[], boolean_atom:[]}], type: Location, kind: 1}], level:[0, 1, 1]}, value: {expressions:[{atom:[{symbol_atom:[], int_atom:[], real_atom:[], boolean_atom:[False]}], type: up:bool, kind: 1}], level:[0]}}"

# Create the action move to change the location

ros2 service call /upf4ros2/srv/add_action upf_msgs/srv/AddAction "{problem_name: test, action:{name: move, parameters:[{name: l_from, type: Location},{name: l_to, type: Location}], duration:[], conditions:[{cond: {expressions:[{atom:[], type: up:bool, kind: 5},{atom:[{symbol_atom:['robot_at'], int_atom:[], real_atom:[], boolean_atom:[]}], type: up:bool, kind: 3},{atom:[{symbol_atom:['l_from'], int_atom:[], real_atom:[], boolean_atom:[]}], type: Location, kind: 2}], level:[0, 1, 1]}, span:[]}], effects:[{effect:{kind: 0, fluent: {expressions:[{atom:[], type: up:bool, kind: 5}, {atom:[{symbol_atom:['robot_at'], int_atom:[], real_atom:[], boolean_atom:[]}], type: up:bool, kind: 3}, {atom:[{symbol_atom:['l_from'], int_atom:[], real_atom:[], boolean_atom:[]}], type: Location, kind: 2}], level:[0, 1, 1]}, value: {expressions:[{atom:[{symbol_atom:[], int_atom:[], real_atom:[], boolean_atom:[False]}], type: up:bool, kind: 1}], level:[0]}, condition:{expressions:[{atom:[{symbol_atom:[], int_atom:[], real_atom:[], boolean_atom:[True]}], type: up:bool, kind: 1}], level:[0]}}, occurrence_time:[]}, {effect:{kind: 0, fluent: {expressions:[{atom:[], type: up:bool, kind: 5}, {atom:[{symbol_atom:['robot_at'], int_atom:[], real_atom:[], boolean_atom:[]}], type: up:bool, kind: 3}, {atom:[{symbol_atom:['l_to'], int_atom:[], real_atom:[], boolean_atom:[]}], type: Location, kind: 2}], level:[0, 1, 1]}, value: {expressions:[{atom:[{symbol_atom:[], int_atom:[], real_atom:[], boolean_atom:[True]}], type: up:bool, kind: 1}], level:[0]}, condition:{expressions:[{atom:[{symbol_atom:[], int_atom:[], real_atom:[], boolean_atom:[True]}], type: up:bool, kind: 1}], level:[0]}}, occurrence_time:[]}]}}"

# Create the goal to the problem
# The robot is at l2 at the end of the problem

ros2 service call /upf4ros2/add_goal upf_msgs/srv/AddGoal "{problem_name: test, goal:[{goal:{expressions:[{atom:[], type: up:bool, kind: 5}, {atom:[{symbol_atom:['robot_at'], int_atom:[], real_atom:[], boolean_atom:[]}], type: up:bool, kind: 3}, {atom:[{symbol_atom:['l2'], int_atom:[], real_atom:[], boolean_atom:[]}], type: Location, kind: 1}], level:[0, 1, 1]}, timing:[]}], goal_with_cost:[]}"

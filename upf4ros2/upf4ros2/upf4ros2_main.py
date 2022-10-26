# Copyright 2022 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rclpy
from rclpy.node import Node

import unified_planning
from unified_planning.model import *


from upf_msgs.srv import AddAction, AddProblem, AddFluent
from upf_msgs.msg import UserType

# from upf_msgs.msg import InstantaneousAction, DurativeAction


class UPF4ROS2Node(Node):

    def __init__(self):
        super().__init__('upf4ros2')

        self.add_problem_srv = self.create_service(
            AddProblem, 'add_fluent', self.add_fluent_callback)
        self.add_action_srv = self.create_service(
            AddAction, 'add_action', self.add_action_callback)
        self.add_problem_srv = self.create_service(
            AddProblem, 'add_problem', self.add_problem_callback)

        self.problems = {}
        self.types = {}

    def add_fluent_callback(self, request, response):
        self.get_logger().info('incoming add_fluent request')

        problem = self.get_problem(request.problem)

        if (problem == None):
            response.success = False
            response.message = 'Problem {} not found to add fluent'.format(request.problem)
        else:
            fluent_type = get_type(request.fluent.type)
            if fluent_type == None:
              fluent_type = add_type(request.fluent.type)

        return response

    def get_type(self, type):
        if type.builtin_type == UserType.BOOL:
            return BoolType()
        elif type.builtin_type == UserType.REAL:
            return RealType()
        elif type.builtin_type == UserType.INTEGER:
            return IntegerType()
        else:
            if type.name in unified_planning.Environment.type_manager:
                return self.types[type.name]
            else:
                new_type = UserType(type.name)
                self.types[type.name] = new_type
                return self.types[type.name]

    def add_action_callback(self, request, response):
        self.get_logger().info('incoming add_action request')

        if len(request.instantaneous_action) == 1:
            (response.success, response.message) = self.add_instantaneous_action(
                request.problem, request.instantaneous_action[0])
        elif len(request.durative_action) == 1:
            (response.success, response.message) = self.add_durative_action(
                request.problem, request.durative_action[0])
        else:
            response.success = False
            response.message = 'wrong add_action request size ({}, {})'.format(
                len(request.instantaneous_action), len(request.durative_action))

        return response

    def add_instantaneous_action(self, problem_name, action):
        action = unified_planning.model.InstantaneousAction(action.name, )
        self.problems[problem_name].add_action(action)
        return (True, '')

    def add_durative_action(self, problem_name, action):
        self.problems[problem_name].add_action(action)
        return (True, '')

    def add_problem_callback(self, request, response):
        self.get_logger().info('incoming add_problem request')

        self.problems[request.problem.name] = unified_planning.model.Problem(request.problem.name)
        return response

    def get_problem(self, name):
        if self.problems.get(name) != None:
            return self.problems[name]
        else:
            return None

def main(args=None):
    rclpy.init(args=args)

    upf4ros2_node = UPF4ROS2Node()

    rclpy.spin(upf4ros2_node)

    upf4ros2_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

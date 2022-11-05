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
#

import tempfile

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from unified_planning.io.pddl_reader import PDDLReader
import unified_planning.model

from upf4ros2.ros2_interface_writer import ROS2InterfaceWriter

from upf_msgs.action import PDDLPlanOneShot
from upf_msgs.msg import PDDLPlanRequest


class UPF4ROS2Node(Node):

    def __init__(self):
        super().__init__('upf4ros2')

        self._ros2_interface_writer = ROS2InterfaceWriter()
        self._pddl_plan_one_shot_server = ActionServer(
            self,
            PDDLPlanOneShot,
            'upf4ros2/planOneShot',
            self.pddl_plan_one_shot_callback)

    def pddl_plan_one_shot_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        domain_file = ''
        problem_file = ''

        if goal_handle.request.plan_request.mode == PDDLPlanRequest.RAW:
            domain_file = tempfile.NamedTemporaryFile()
            problem_file = tempfile.NamedTemporaryFile()

            with open(domain_file, 'w') as pddl_writer:
                pddl_writer.write(goal_handle.request.plan_request.domain)
            with open(problem_file, 'w') as pddl_writer:
                pddl_writer.write(goal_handle.request.plan_request.problem)
        else:
            domain_file = goal_handle.request.plan_request.domain
            problem_file = goal_handle.request.plan_request.problem

        reader = PDDLReader()
        upf_problem = reader.parse_problem(domain_file, problem_file)

        with unified_planning.shortcuts.OneshotPlanner(problem_kind=upf_problem.kind) as planner:
            result = planner.solve(upf_problem)
            print('%s returned: %s' % (planner.name, result.plan))

            feedback_msg = PDDLPlanOneShot.Feedback()
            feedback_msg.plan_result = self._ros2_interface_writer.convert(result)

            goal_handle.publish_feedback(feedback_msg)
            goal_handle.succeed()

            result = PDDLPlanOneShot.Result()
            result.success = True
            result.message = ''
            return result


def main(args=None):
    rclpy.init(args=args)

    upf4ros2_node = UPF4ROS2Node()

    rclpy.spin(upf4ros2_node)

    upf4ros2_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

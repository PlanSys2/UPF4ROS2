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

import unittest

import rclpy
import rclpy.executors
from rclpy.duration import Duration

from time import sleep

from upf4ros2 import upf4ros2_main
from upf_msgs.srv import AddAction, AddProblem


class TestUPF4ROS2(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node('TestUPF4ROS2', context=cls.context)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def test_void_problem(self):
        rclpy.init(args=None)

        node_cli = rclpy.create_node('test_void_problem_node')
        node_test = upf4ros2_main.UPF4ROS2Node()

        cli = node_cli.create_client(AddProblem, 'add_problem')

        rclpy.spin_once(node_test, timeout_sec=0.01)
        while not cli.wait_for_service(timeout_sec=0.1):
          rclpy.spin_once(node_test)
          node_cli.get_logger().info('service not available, waiting again...')


        req = AddProblem.Request()
        req.problem.name = 'problem1'

        future = cli.call_async(req)
        start = node_test.get_clock().now()
        while rclpy.ok() and (node_test.get_clock().now() - start) < Duration(seconds=1):
          rclpy.spin_once(node_test, timeout_sec=0.01)
          rclpy.spin_once(node_cli, timeout_sec=0.01)
          sleep(0.02)

        p1 = node_test.get_problem('problem2')
        self.assertEqual(p1, None)
        p2 = node_test.get_problem('problem1')
        self.assertNotEqual(p2, None)
        self.assertEqual(p2.name, 'problem1')
        self.assertEqual(len(p2.actions), 0)
        self.assertEqual(len(p2.user_types), 0)
        self.assertEqual(len(p2.fluents), 0)
        self.assertEqual(len(p2.timed_effects), 0)
        self.assertEqual(len(p2.timed_goals), 0)
        self.assertEqual(len(p2.goals), 0)
        self.assertEqual(len(p2.quality_metrics), 0)

        node_cli.destroy_node()
        node_test.destroy_node()
        rclpy.shutdown()

    def test_add_action(self):
        rclpy.init(args=None)

        node_cli = rclpy.create_node('test_void_problem_node')
        node_test = upf4ros2_main.UPF4ROS2Node()

        add_problem_cli = node_cli.create_client(AddProblem, 'add_problem')
        add_action_cli = node_cli.create_client(AddAction, 'add_action')

        rclpy.spin_once(node_test, timeout_sec=0.01)
        while not add_problem_cli.wait_for_service(timeout_sec=0.1):
          rclpy.spin_once(node_test)
          node_cli.get_logger().info('service add_problem not available, waiting again...')

        rclpy.spin_once(node_test, timeout_sec=0.01)
        while not add_action_cli.wait_for_service(timeout_sec=0.1):
          rclpy.spin_once(node_test)
          node_cli.get_logger().info('service add_action not available, waiting again...')


        add_problem_req = AddProblem.Request()
        add_problem_req.problem.name = 'problem1'

        future = add_problem_cli.call_async(add_problem_req)
        start = node_test.get_clock().now()
        while rclpy.ok() and (node_test.get_clock().now() - start) < Duration(seconds=1):
          rclpy.spin_once(node_test, timeout_sec=0.01)
          rclpy.spin_once(node_cli, timeout_sec=0.01)
          sleep(0.02)
      
        req_wrong_1 = AddAction.Request()
        req_wrong_1.problem = 'noexist'

        problem = node_test.get_problem('problem1')
        self.assertNotEqual(problem, None)
        self.assertEqual(problem.name, 'problem1')

        node_cli.destroy_node()
        node_test.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    unittest.main()

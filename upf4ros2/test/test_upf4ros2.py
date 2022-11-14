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

import threading
import unittest

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.action import ActionClient
import rclpy.executors
# from rclpy.duration import Duration

# from time import sleep

from unified_planning import shortcuts
from unified_planning.io.pddl_reader import PDDLReader
from upf4ros2 import upf4ros2_main
from unified_planning.test.examples import get_example_problems

# from upf_msgs.srv import AddAction, AddProblem
from upf4ros2.ros2_interface_reader import ROS2InterfaceReader  # type: ignore[attr-defined]
from upf4ros2.ros2_interface_writer import ROS2InterfaceWriter  # type: ignore[attr-defined]
from upf_msgs.action import (
    PDDLPlanOneShot,
    PlanOneShot
)
from upf_msgs.srv import (
    AddAction,
    AddFluent,
    AddGoal,
    AddObject,
    NewProblem,
    SetInitialValue,
    SetProblem
)
from upf_msgs.msg import (
    PDDLPlanRequest,
    PlanRequest
)


class TestUPF4ROS2(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node('TestUPF4ROS2', context=cls.context)
        shortcuts.get_env().credits_stream = None

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def test_plan_from_file_pddl_no_tt(self):
        rclpy.init(args=None)

        executor = rclpy.executors.SingleThreadedExecutor()
        node_test = upf4ros2_main.UPF4ROS2Node()
        node_cli = rclpy.create_node('test_plan_from_file_pddl_no_tt_node')
        executor.add_node(node_test)
        executor.add_node(node_cli)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        goal_msg = PDDLPlanOneShot.Goal()
        goal_msg.plan_request.mode = PDDLPlanRequest.FILE
        goal_msg.plan_request.domain = (get_package_share_directory('upf4ros2')
                                        + '/pddl/gripper_domain.pddl')
        goal_msg.plan_request.problem = (get_package_share_directory('upf4ros2')
                                         + '/pddl/gripper_problem_0.pddl')

        reader = PDDLReader()
        upf_problem = reader.parse_problem(
            goal_msg.plan_request.domain,
            goal_msg.plan_request.problem)

        client = ActionClient(node_cli, PDDLPlanOneShot, 'upf4ros2/planOneShotPDDL')

        def goal_response_callback(future):
            goal_handle = future.result()
            self.assertTrue(goal_handle.accepted)
            if not goal_handle.accepted:
                node_cli.get_logger().info('Goal rejected :(')
                return

            node_cli.get_logger().info('Goal accepted :)')

            node_cli._get_result_future = goal_handle.get_result_async()
            node_cli._get_result_future.add_done_callback(get_result_callback)

        def get_result_callback(future):
            result = future.result().result
            self.assertEqual(result.success, True)
            self.assertEqual(result.message, '')

            node_cli.get_logger().info('Result: success: {0} message:{1}'.
                                       format(result.success, result.message))
            rclpy.shutdown()

        def feedback_callback(feedback_msg):
            feedback = feedback_msg.feedback
            pb_reader = ROS2InterfaceReader()
            upf_plan = pb_reader.convert(feedback.plan_result.plan, upf_problem)
            node_cli.get_logger().info('Received feedback: {0}'.
                                       format(upf_plan))

            good_plan = '[pick(ball1, rooma, left), move(rooma, roomb), drop(ball1, roomb, left)]'
            self.assertEqual(upf_plan.__repr__(), good_plan)

        client.wait_for_server()

        send_goal_future = client.send_goal_async(goal_msg, feedback_callback=feedback_callback)
        send_goal_future.add_done_callback(goal_response_callback)

        executor_thread.join()

    def test_plan_from_file_pddl_tt(self):
        rclpy.init(args=None)

        executor = rclpy.executors.SingleThreadedExecutor()
        node_test = upf4ros2_main.UPF4ROS2Node()
        node_cli = rclpy.create_node('test_plan_from_file_pddl_tt_node')
        executor.add_node(node_test)
        executor.add_node(node_cli)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        goal_msg = PDDLPlanOneShot.Goal()
        goal_msg.plan_request.mode = PDDLPlanRequest.FILE
        goal_msg.plan_request.domain = (get_package_share_directory('upf4ros2')
                                        + '/pddl/domain_tt.pddl')
        goal_msg.plan_request.problem = (get_package_share_directory('upf4ros2')
                                         + '/pddl/problem_tt_1.pddl')

        reader = PDDLReader()
        upf_problem = reader.parse_problem(
            goal_msg.plan_request.domain,
            goal_msg.plan_request.problem)

        client = ActionClient(node_cli, PDDLPlanOneShot, 'upf4ros2/planOneShotPDDL')

        def goal_response_callback(future):
            goal_handle = future.result()
            self.assertTrue(goal_handle.accepted)
            if not goal_handle.accepted:
                node_cli.get_logger().info('Goal rejected :(')
                return

            node_cli.get_logger().info('Goal accepted :)')

            node_cli._get_result_future = goal_handle.get_result_async()
            node_cli._get_result_future.add_done_callback(get_result_callback)

        def get_result_callback(future):
            result = future.result().result
            self.assertEqual(result.success, True)
            self.assertEqual(result.message, '')

            node_cli.get_logger().info('Result: success: {0} message:{1}'.
                                       format(result.success, result.message))
            rclpy.shutdown()

        def feedback_callback(feedback_msg):
            feedback = feedback_msg.feedback
            pb_reader = ROS2InterfaceReader()
            upf_plan = pb_reader.convert(feedback.plan_result.plan, upf_problem)
            node_cli.get_logger().info('Received feedback: {0}'.
                                       format(upf_plan))

            good_plan = '[(Fraction(0, 1), move(leia, kitchen, bedroom), Fraction(5, 1))]'
            self.assertEqual(upf_plan.__repr__(), good_plan)

        client.wait_for_server()

        send_goal_future = client.send_goal_async(goal_msg, feedback_callback=feedback_callback)
        send_goal_future.add_done_callback(goal_response_callback)

        executor_thread.join()

    def test_plan_robot(self):
        rclpy.init(args=None)

        executor = rclpy.executors.SingleThreadedExecutor()
        node_test = upf4ros2_main.UPF4ROS2Node()
        node_cli = rclpy.create_node('test_plan_robot_node')
        executor.add_node(node_test)
        executor.add_node(node_cli)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        problems = get_example_problems()
        problem = problems['robot'].problem

        pb_writter = ROS2InterfaceWriter()

        goal_msg = PlanOneShot.Goal()
        goal_msg.plan_request.problem = pb_writter.convert(problem)

        client = ActionClient(node_cli, PlanOneShot, 'upf4ros2/planOneShot')

        def goal_response_callback(future):
            goal_handle = future.result()
            self.assertTrue(goal_handle.accepted)
            if not goal_handle.accepted:
                node_cli.get_logger().info('Goal rejected :(')
                return

            node_cli.get_logger().info('Goal accepted :)')

            node_cli._get_result_future = goal_handle.get_result_async()
            node_cli._get_result_future.add_done_callback(get_result_callback)

        def get_result_callback(future):
            result = future.result().result
            self.assertEqual(result.success, True)
            self.assertEqual(result.message, '')

            node_cli.get_logger().info('Result: success: {0} message:{1}'.
                                       format(result.success, result.message))
            rclpy.shutdown()

        def feedback_callback(feedback_msg):
            feedback = feedback_msg.feedback
            pb_reader = ROS2InterfaceReader()
            upf_plan = pb_reader.convert(feedback.plan_result.plan, problem)
            node_cli.get_logger().info('Received feedback: {0}'.
                                       format(upf_plan))

            good_plan = '[move(l1, l2)]'
            self.assertEqual(upf_plan.__repr__(), good_plan)

        client.wait_for_server()

        send_goal_future = client.send_goal_async(goal_msg, feedback_callback=feedback_callback)
        send_goal_future.add_done_callback(goal_response_callback)

        executor_thread.join()

    def test_new_problem(self):
        rclpy.init(args=None)

        executor = rclpy.executors.SingleThreadedExecutor()
        node_test = upf4ros2_main.UPF4ROS2Node()
        node_cli = rclpy.create_node('test_new_problem')
        executor.add_node(node_test)
        executor.add_node(node_cli)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        client = node_cli.create_client(NewProblem, 'upf4ros2/new_problem')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        srv = NewProblem.Request()
        srv.problem_name = "problem_test_1"
        
        response = client.call(srv)
        self.assertTrue(response.success)
        self.assertEqual(response.message, '')

        response = client.call(srv)
        self.assertFalse(response.success)
        self.assertEqual(response.message, 'Problem problem_test_1 already exists')

        rclpy.shutdown()
        executor_thread.join()

if __name__ == '__main__':
    unittest.main()

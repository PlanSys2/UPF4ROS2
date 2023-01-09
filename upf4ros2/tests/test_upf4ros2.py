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


import threading
import unittest

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.action import ActionClient
import rclpy.executors
# from rclpy.duration import Duration

# from time import sleep

from unified_planning import model
from unified_planning import shortcuts
from unified_planning.io.pddl_reader import PDDLReader
from unified_planning.test.examples import get_example_problems
from upf4ros2 import upf4ros2_main

# from upf_msgs.srv import AddAction, AddProblem
from upf4ros2.ros2_interface_reader import ROS2InterfaceReader  # type: ignore[attr-defined]
from upf4ros2.ros2_interface_writer import ROS2InterfaceWriter  # type: ignore[attr-defined]
from upf_msgs import msg as msgs
from upf_msgs.action import (
    PDDLPlanOneShot,
    PlanOneShot
)
from upf_msgs.srv import (
    AddAction,
    AddFluent,
    AddGoal,
    AddObject,
    GetProblem,
    NewProblem,
    SetInitialValue,
    SetProblem
)
from upf_msgs.srv import PDDLPlanOneShot as PDDLPlanOneShotSrv


def spin_srv(executor):
    try:
        executor.spin()
    except rclpy.executors.ExternalShutdownException:
        pass


def call_srv_manually(client_node):
    client_node.call_srv()
    client_node.get_logger().info('Test finished successfully.\n')


class TestUPF4ROS2(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

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
        goal_msg.plan_request.mode = msgs.PDDLPlanRequest.FILE
        goal_msg.plan_request.domain = (get_package_share_directory('upf4ros2')
                                        + '/pddl/gripper_domain.pddl')
        goal_msg.plan_request.problem = (get_package_share_directory('upf4ros2')
                                         + '/pddl/gripper_problem_0.pddl')

        reader = PDDLReader()
        upf_problem = reader.parse_problem(
            goal_msg.plan_request.domain,
            goal_msg.plan_request.problem)

        client = ActionClient(node_cli, PDDLPlanOneShot, 'upf4ros2/action/planOneShotPDDL')

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
            node_cli.destroy_node()
            node_test.destroy_node()
            executor.shutdown()
            rclpy.shutdown()
            executor_thread.join()

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

        node_cli.destroy_node()
        node_test.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
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
        goal_msg.plan_request.mode = msgs.PDDLPlanRequest.FILE
        goal_msg.plan_request.domain = (get_package_share_directory('upf4ros2')
                                        + '/pddl/domain_tt.pddl')
        goal_msg.plan_request.problem = (get_package_share_directory('upf4ros2')
                                         + '/pddl/problem_tt_1.pddl')

        reader = PDDLReader()
        upf_problem = reader.parse_problem(
            goal_msg.plan_request.domain,
            goal_msg.plan_request.problem)

        client = ActionClient(node_cli, PDDLPlanOneShot, 'upf4ros2/action/planOneShotPDDL')

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

        node_cli.destroy_node()
        node_test.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
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

        client = ActionClient(node_cli, PlanOneShot, 'upf4ros2/action/planOneShot')

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

        node_cli.destroy_node()
        node_test.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        executor_thread.join()

    def test_plan_from_file_pddl_tt_service(self):
        rclpy.init(args=None)

        executor = rclpy.executors.SingleThreadedExecutor()
        node_test = upf4ros2_main.UPF4ROS2Node()
        node_cli = rclpy.create_node('test_plan_from_file_pddl_tt_service')
        executor.add_node(node_test)
        executor.add_node(node_cli)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        client = node_cli.create_client(PDDLPlanOneShotSrv, 'upf4ros2/srv/planOneShotPDDL')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        srv = PDDLPlanOneShotSrv.Request()
        srv.plan_request.mode = msgs.PDDLPlanRequest.FILE
        srv.plan_request.domain = (get_package_share_directory('upf4ros2')
                                        + '/pddl/domain_tt.pddl')
        srv.plan_request.problem = (get_package_share_directory('upf4ros2')
                                         + '/pddl/problem_tt_1.pddl')

        reader = PDDLReader()
        upf_problem = reader.parse_problem(
            srv.plan_request.domain,
            srv.plan_request.problem)

        response = client.call(srv)
        
        pb_reader = ROS2InterfaceReader()
        upf_plan = pb_reader.convert(response.plan_result.plan, upf_problem)
        node_cli.get_logger().info('Received feedback: {0}'.
                                   format(upf_plan))

        good_plan = '[(Fraction(0, 1), move(leia, kitchen, bedroom), Fraction(5, 1))]'
        self.assertEqual(upf_plan.__repr__(), good_plan)
        self.assertTrue(response.success)
        self.assertEqual(response.message, '')

        node_cli.destroy_node()
        node_test.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
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

        client = node_cli.create_client(NewProblem, 'upf4ros2/srv/new_problem')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        srv = NewProblem.Request()
        srv.problem_name = 'problem_test_1'

        response = client.call(srv)
        self.assertTrue(response.success)
        self.assertEqual(response.message, '')

        srv = NewProblem.Request()
        srv.problem_name = 'problem_test_1'
        response = client.call(srv)
        self.assertFalse(response.success)
        self.assertEqual(response.message, 'Problem problem_test_1 already exists')

        node_cli.destroy_node()
        node_test.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        executor_thread.join()

    def test_set_get_problem(self):
        rclpy.init(args=None)

        executor = rclpy.executors.SingleThreadedExecutor()
        node_test = upf4ros2_main.UPF4ROS2Node()
        node_cli = rclpy.create_node('test_set_get_problem')
        executor.add_node(node_test)
        executor.add_node(node_cli)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        client = node_cli.create_client(SetProblem, 'upf4ros2/srv/set_problem')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        pb_writer = ROS2InterfaceWriter()

        problems = get_example_problems()
        problem = problems['robot'].problem
        srv = SetProblem.Request()
        srv.problem_name = 'problem_test_robot'
        srv.problem = pb_writer.convert(problem)

        response = client.call(srv)
        self.assertTrue(response.success)
        self.assertEqual(response.message, '')

        srv = SetProblem.Request()
        srv.problem_name = 'problem_test_robot'
        srv.problem = pb_writer.convert(problem)

        response = client.call(srv)
        self.assertFalse(response.success)
        self.assertEqual(response.message, 'Problem problem_test_robot already exists')

        client2 = node_cli.create_client(GetProblem, 'upf4ros2/srv/get_problem')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        pb_reader = ROS2InterfaceReader()

        srv2 = GetProblem.Request()
        srv2.problem_name = 'problem_test_robot'

        response2 = client2.call(srv2)
        self.assertTrue(response2.success)

        problem_ret = pb_reader.convert(response2.problem)

        self.assertEqual(problem, problem_ret)

        node_cli.destroy_node()
        node_test.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        executor_thread.join()

    def test_add_set_fluent(self):
        rclpy.init(args=None)

        executor = rclpy.executors.SingleThreadedExecutor()
        node_test = upf4ros2_main.UPF4ROS2Node()
        node_cli = rclpy.create_node('test_add_set_fluent_problem')
        executor.add_node(node_test)
        executor.add_node(node_cli)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        client = node_cli.create_client(SetProblem, 'upf4ros2/srv/set_problem')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        pb_writer = ROS2InterfaceWriter()

        problems = get_example_problems()
        problem = problems['robot'].problem
        srv = SetProblem.Request()
        srv.problem_name = 'problem_test_robot'
        srv.problem = pb_writer.convert(problem)

        response = client.call(srv)
        self.assertTrue(response.success)
        self.assertEqual(response.message, '')

        # Make changes in local and request in global, and check for diffs

        add_fluent_cli = node_cli.create_client(AddFluent, 'upf4ros2/srv/add_fluent')

        Location = shortcuts.UserType('Location')
        robot_at = model.Fluent(
            'robot_at_bis', shortcuts.BoolType(),
            l=Location)

        add_fluent_srv = AddFluent.Request()
        add_fluent_srv.problem_name = 'problem_test_robot'
        add_fluent_srv.fluent = pb_writer.convert(robot_at, problem)

        item = msgs.ExpressionItem()
        item.atom.append(msgs.Atom())
        item.atom[0].boolean_atom.append(False)
        item.type = 'up:bool'
        item.kind = msgs.ExpressionItem.CONSTANT
        value = msgs.Expression()
        value.expressions.append(item)
        value.level.append(0)

        add_fluent_srv.default_value = value

        add_fluent_response = add_fluent_cli.call(add_fluent_srv)
        self.assertTrue(add_fluent_response.success)
        self.assertEqual(add_fluent_response.message, '')

        problem.add_fluent(robot_at, default_initial_value=False)

        set_initial_value_cli = node_cli.create_client(
            SetInitialValue, 'upf4ros2/srv/set_initial_value')
        set_initial_value_srv = SetInitialValue.Request()
        set_initial_value_srv.problem_name = 'problem_test_robot'
        l2 = model.Object('l2', Location)
        set_initial_value_srv.expression = pb_writer.convert(robot_at(l2))
        set_initial_value_srv.value = value

        set_initial_value_response = set_initial_value_cli.call(set_initial_value_srv)
        self.assertTrue(set_initial_value_response.success)
        self.assertEqual(set_initial_value_response.message, '')

        problem.set_initial_value(robot_at(l2), False)

        add_goal_cli = node_cli.create_client(AddGoal, 'upf4ros2/srv/add_goal')
        add_goal_srv = AddGoal.Request()
        add_goal_srv.problem_name = 'problem_test_robot'
        l1 = model.Object('l1', Location)
        add_goal_srv.goal.append(msgs.Goal())
        add_goal_srv.goal[0].goal = pb_writer.convert(robot_at(l1))

        add_goal_response = add_goal_cli.call(add_goal_srv)
        self.assertTrue(add_goal_response.success)
        self.assertEqual(add_goal_response.message, '')

        problem.add_goal(robot_at(l1))

        ###############################################################

        client2 = node_cli.create_client(GetProblem, 'upf4ros2/srv/get_problem')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        pb_reader = ROS2InterfaceReader()

        srv2 = GetProblem.Request()
        srv2.problem_name = 'problem_test_robot'

        response2 = client2.call(srv2)
        self.assertTrue(response2.success)

        problem_ret = pb_reader.convert(response2.problem)

        self.assertEqual(problem, problem_ret)

        node_cli.destroy_node()
        node_test.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        executor_thread.join()

    def test_add_action(self):
        rclpy.init(args=None)

        executor = rclpy.executors.SingleThreadedExecutor()
        node_test = upf4ros2_main.UPF4ROS2Node()
        node_cli = rclpy.create_node('test_add_action')
        executor.add_node(node_test)
        executor.add_node(node_cli)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        client = node_cli.create_client(SetProblem, 'upf4ros2/srv/set_problem')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        pb_writer = ROS2InterfaceWriter()

        problems = get_example_problems()
        problem = problems['robot'].problem
        srv = SetProblem.Request()
        srv.problem_name = 'problem_test_robot'
        srv.problem = pb_writer.convert(problem)

        response = client.call(srv)
        self.assertTrue(response.success)
        self.assertEqual(response.message, '')

        # Make changes in local and request in global, and check for diffs

        add_action_cli = node_cli.create_client(AddAction, 'upf4ros2/srv/add_action')

        Location = shortcuts.UserType('Location')
        robot_at = model.Fluent('robot_at', shortcuts.BoolType(), l=Location)

        move = model.InstantaneousAction('move2', l_from=Location, l_to=Location)
        l_from = move.parameter('l_from')
        l_to = move.parameter('l_to')
        move.add_precondition(robot_at(l_from))
        move.add_effect(robot_at(l_from), False)
        move.add_effect(robot_at(l_to), True)

        add_action_srv = AddAction.Request()
        add_action_srv.problem_name = 'problem_test_robot'
        add_action_srv.action = pb_writer.convert(move)

        add_action_response = add_action_cli.call(add_action_srv)
        self.assertTrue(add_action_response.success)
        self.assertEqual(add_action_response.message, '')

        problem.add_action(move)

        ###############################################################

        client2 = node_cli.create_client(GetProblem, 'upf4ros2/srv/get_problem')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        # pb_reader = ROS2InterfaceReader()

        srv2 = GetProblem.Request()
        srv2.problem_name = 'problem_test_robot'

        response2 = client2.call(srv2)
        self.assertTrue(response2.success)

        # problem_ret = pb_reader.convert(response2.problem)
        # self.assertEqual(problem, problem_ret)

        node_cli.destroy_node()
        node_test.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        executor_thread.join()

    def test_add_object(self):
        rclpy.init(args=None)

        executor = rclpy.executors.SingleThreadedExecutor()
        node_test = upf4ros2_main.UPF4ROS2Node()
        node_cli = rclpy.create_node('test_add_object')
        executor.add_node(node_test)
        executor.add_node(node_cli)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        client = node_cli.create_client(SetProblem, 'upf4ros2/srv/set_problem')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        pb_writer = ROS2InterfaceWriter()

        problems = get_example_problems()
        problem = problems['robot'].problem
        srv = SetProblem.Request()
        srv.problem_name = 'problem_test_robot'
        srv.problem = pb_writer.convert(problem)

        response = client.call(srv)
        self.assertTrue(response.success)
        self.assertEqual(response.message, '')

        # Make changes in local and request in global, and check for diffs

        add_object_cli = node_cli.create_client(AddObject, 'upf4ros2/srv/add_object')

        Location = shortcuts.UserType('Location')

        upf_object = model.Object('l3', Location)

        add_object_srv = AddObject.Request()
        add_object_srv.problem_name = 'problem_test_robot'
        add_object_srv.object = pb_writer.convert(upf_object)

        add_object_response = add_object_cli.call(add_object_srv)
        self.assertTrue(add_object_response.success)
        self.assertEqual(add_object_response.message, '')

        problem.add_object(upf_object)

        ###############################################################

#        client2 = node_cli.create_client(GetProblem, 'upf4ros2/get_problem')
#        while not client.wait_for_service(timeout_sec=1.0):
#            self.get_logger().info('service not available, waiting again...')
#
#        pb_reader = ROS2InterfaceReader()
#
#        srv2 = GetProblem.Request()
#        srv2.problem_name = 'problem_test_robot'
#
#        response2 = client2.call(srv2)
#        self.assertTrue(response2.success)
#
#        problem_ret = pb_reader.convert(response2.problem)

        # self.assertEqual(problem, problem_ret)
        node_cli.destroy_node()
        node_test.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        executor_thread.join()


if __name__ == '__main__':
    unittest.main()

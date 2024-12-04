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

# from unified_planning import model
# from unified_planning import shortcuts
from unified_planning.io.pddl_reader import PDDLReader
# from unified_planning.test.examples import get_example_problems
from upf4ros2 import upf4ros2_main

# from upf_msgs.srv import AddAction, AddProblem
# type: ignore[attr-defined]
from upf4ros2.ros2_interface_reader import ROS2InterfaceReader
# from upf4ros2.ros2_interface_writer import ROS2InterfaceWriter  # type:
# ignore[attr-defined]
from upf_msgs import msg as msgs
from upf_msgs.action import (
    PDDLPlanOneShot
)


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
        goal_msg.plan_request.problem = (
            get_package_share_directory('upf4ros2') +
            '/pddl/gripper_problem_0.pddl')

        reader = PDDLReader()
        upf_problem = reader.parse_problem(
            goal_msg.plan_request.domain,
            goal_msg.plan_request.problem)

        client = ActionClient(
            node_cli,
            PDDLPlanOneShot,
            'upf4ros2/action/planOneShotPDDL')

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
            upf_plan = pb_reader.convert(
                feedback.plan_result.plan, upf_problem)
            node_cli.get_logger().info('Received feedback: {0}'.
                                       format(upf_plan))

            good_plan = '[pick(ball1, rooma, left), move(rooma, roomb), drop(ball1, roomb, left)]'
            self.assertEqual(upf_plan.__repr__(), good_plan)

        client.wait_for_server()

        send_goal_future = client.send_goal_async(
            goal_msg, feedback_callback=feedback_callback)
        send_goal_future.add_done_callback(goal_response_callback)

        node_cli.destroy_node()
        node_test.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        executor_thread.join()


if __name__ == '__main__':
    unittest.main()

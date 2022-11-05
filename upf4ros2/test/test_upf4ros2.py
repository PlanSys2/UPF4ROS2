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
import threading

import rclpy
import rclpy.executors
from rclpy.duration import Duration
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory

from time import sleep

from upf4ros2 import upf4ros2_main
from upf_msgs.srv import AddAction, AddProblem
from upf_msgs.action import PDDLPlanOneShot
from upf_msgs.msg import PDDLPlanRequest

from upf4ros2.ros2_interface_reader import ROS2InterfaceReader  # type: ignore[attr-defined]
from upf4ros2.ros2_interface_writer import ROS2InterfaceWriter  # type: ignore[attr-defined]

import unified_planning
from unified_planning.io.pddl_reader import PDDLReader

class TestUPF4ROS2(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node('TestUPF4ROS2', context=cls.context)
        unified_planning.shortcuts.get_env().credits_stream = None

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)

    def test_plan_from_file_pddl_no_tt(self):
        rclpy.init(args=None)

        executor = rclpy.executors.SingleThreadedExecutor()
        node_test = upf4ros2_main.UPF4ROS2Node()
        node_cli = rclpy.create_node('test_generate_plan_from_file_pddl_node')
        executor.add_node(node_test)
        executor.add_node(node_cli)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        goal_msg = PDDLPlanOneShot.Goal()
        goal_msg.plan_request.mode = PDDLPlanRequest.FILE
        goal_msg.plan_request.domain = get_package_share_directory('upf4ros2') + '/pddl/gripper_domain.pddl'
        goal_msg.plan_request.problem = get_package_share_directory('upf4ros2') + '/pddl/gripper_problem_0.pddl'

        reader = PDDLReader()
        upf_problem = reader.parse_problem(goal_msg.plan_request.domain, goal_msg.plan_request.problem)

        client = ActionClient(node_cli, PDDLPlanOneShot, 'upf4ros2/planOneShot')
        
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

if __name__ == '__main__':
    unittest.main()

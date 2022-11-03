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

if __name__ == '__main__':
    unittest.main()

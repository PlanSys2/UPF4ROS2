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




class UPF4ROS2Node(Node):

    def __init__(self):
        super().__init__('upf4ros2')


def main(args=None):
    rclpy.init(args=args)

    upf4ros2_node = UPF4ROS2Node()

    rclpy.spin(upf4ros2_node)

    upf4ros2_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

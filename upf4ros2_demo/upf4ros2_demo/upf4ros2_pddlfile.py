import tempfile
import threading
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from unified_planning import model
from unified_planning.io.pddl_reader import PDDLReader

from upf4ros2.ros2_interface_reader import ROS2InterfaceReader
from upf4ros2.ros2_interface_writer import ROS2InterfaceWriter
from unified_planning import model
from unified_planning import shortcuts
from upf_msgs import msg as msgs

from upf_msgs.action import (
    PDDLPlanOneShot,
)
from upf_msgs.srv import (
    GetProblem,
)

class UPF4ROS2PDDLNode(Node):

    def __init__(self):
        super().__init__('upf4ros2_pddl')

        self.declare_parameter('domain', '/pddl/domain.pddl')
        self.declare_parameter('problem', '/pddl/problem.pddl')

        self._domain = self.get_parameter('domain')
        self._problem = self.get_parameter('problem')

        self._ros2_interface_writer = ROS2InterfaceWriter()
        self._ros2_interface_reader = ROS2InterfaceReader()

        self._plan_pddl_one_shot_client = ActionClient(
            self, 
            PDDLPlanOneShot, 
            'upf4ros2/planOneShotPDDL')

        self._get_problem = self.create_client(
            GetProblem, 'upf4ros2/get_problem')

    def get_problem(self):
        srv = GetProblem.Request()
        srv.problem_name = self._problem_name

        self._get_problem.wait_for_service()
        self.future = self._get_problem.call_async(srv)
        rclpy.spin_until_future_complete(self, self.future)
        problem = self._ros2_interface_reader.convert(self.future.result().problem)
        return problem

    def get_plan(self):
        
        self.get_logger().info('Planning...')
        goal_msg = PDDLPlanOneShot.Goal()
        goal_msg.plan_request.mode = msgs.PDDLPlanRequest.FILE
        goal_msg.plan_request.domain = (get_package_share_directory('upf4ros2_demo')
                                        + str(self._domain.value))
        goal_msg.plan_request.problem = (get_package_share_directory('upf4ros2_demo')
                                        + str(self._problem.value))

        self._plan_pddl_one_shot_client.wait_for_server()

        res = self._plan_pddl_one_shot_client.send_goal_async(goal_msg)
        res.add_done_callback(self.goal_response_callback)

        return res

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Solution not found :(')
            return

        self.get_logger().info('Solution found :)')

        

def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.SingleThreadedExecutor()
    upf4ros2_pddl_node = UPF4ROS2PDDLNode()

    executor.add_node(upf4ros2_pddl_node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    res = upf4ros2_pddl_node.get_plan()
    rclpy.spin(upf4ros2_pddl_node)

    upf4ros2_pddl_node.destroy_node()
    executor.shutdown()
    rclpy.shutdown()
    executor_thread.join()


if __name__ == '__main__':
    main()

import tempfile
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from unified_planning import model
from unified_planning.io.pddl_reader import PDDLReader
from unified_planning.shortcuts import OneshotPlanner

from upf4ros2.ros2_interface_reader import ROS2InterfaceReader
from upf4ros2.ros2_interface_writer import ROS2InterfaceWriter
from unified_planning import model
from unified_planning import shortcuts
from upf_msgs import msg as msgs

from upf_msgs.action import (
    PDDLPlanOneShot,
    PlanOneShot
)

from upf_msgs.msg import (
    PDDLPlanRequest
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


class UPF4ROS2PlanNode(Node):

    def __init__(self):
        super().__init__('upf4ros2_plan')

        self.declare_parameter('problem_name', 'test')
        self._problem_name = self.get_parameter('problem_name')

        self._ros2_interface_writer = ROS2InterfaceWriter()
        self._ros2_interface_reader = ROS2InterfaceReader()

        self._plan_one_shot_client = ActionClient(
            self,
            PlanOneShot,
            'upf4ros2/planOneShot')

        self._get_problem = self.create_client(
            GetProblem, 'upf4ros2/get_problem')

    def get_problem(self):
        srv = GetProblem.Request()
        srv.problem_name = str(self._problem_name.value)

        self._get_problem.wait_for_service()
        self.future = self._get_problem.call_async(srv)
        rclpy.spin_until_future_complete(self, self.future)
        problem = self._ros2_interface_reader.convert(self.future.result().problem)
        return problem

    def get_plan(self):
        self.get_logger().info('Planning...')
        problem = self.get_problem()
        goal_msg = PlanOneShot.Goal()
        goal_msg.plan_request.problem = self._ros2_interface_writer.convert(problem)

        self._plan_one_shot_client.wait_for_server()
        self.future = self._plan_one_shot_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Solution not found :(')
            return

        self.get_logger().info('Solution found :)')


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        #[self.get_logger().info(f'{i}') for i in feedback.plan_result.plan.actions]

        

def main(args=None):
    rclpy.init(args=args)

    upf4ros2_plan_node = UPF4ROS2PlanNode()

    problem = upf4ros2_plan_node.get_problem()
    upf4ros2_plan_node.get_logger().info(f'{problem}')

    upf4ros2_plan_node.get_plan()
    rclpy.spin(upf4ros2_plan_node)

    upf4ros2_plan_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

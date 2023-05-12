import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

from upf4ros2.ros2_interface_reader import ROS2InterfaceReader
from upf4ros2.ros2_interface_writer import ROS2InterfaceWriter
from upf_msgs import msg as msgs
from upf4ros2_demo_msgs.srv import CallAction

from upf_msgs.action import (
    PDDLPlanOneShot,
    PlanOneShot
)

from upf_msgs.srv import (
    GetProblem
)

from upf_msgs.srv import PDDLPlanOneShot as PDDLPlanOneShotSrv

class UPF4ROS2DemoNode(Node):

    def __init__(self):
        super().__init__('plan_executor')

        self._problem_name = ''
        self._plan_result = {}

        self.declare_parameter('domain', '/pddl/uav_domain.pddl')
        self.declare_parameter('problem', '/pddl/generated_uav_instance.pddl')

        self._domain = self.get_parameter('domain')
        self._problem = self.get_parameter('problem')

        self._ros2_interface_writer = ROS2InterfaceWriter()
        self._ros2_interface_reader = ROS2InterfaceReader()

        self._plan_pddl_one_shot_client = ActionClient(
            self, 
            PDDLPlanOneShot, 
            'upf4ros2/action/planOneShotPDDL')

        self._get_problem = self.create_client(
            GetProblem, 'upf4ros2/srv/get_problem')
        self._plan_pddl_one_shot_client_srv = self.create_client(
            PDDLPlanOneShotSrv, 'upf4ros2/srv/planOneShotPDDL')  
    

        

    def get_problem(self):
        srv = GetProblem.Request()
        srv.problem_name = self._problem_name

        self._get_problem.wait_for_service()
        self.future = self._get_problem.call_async(srv)
        rclpy.spin_until_future_complete(self, self.future)
        # problem = self._ros2_interface_reader.convert(self.future.result().problem)
        problem = self.future.result().problem
        return problem

    def get_plan_srv(self):
        
        problem = self.get_problem()

        self.get_logger().info('Planning...')
        srv = PDDLPlanOneShotSrv.Request()
        srv.plan_request.mode = msgs.PDDLPlanRequest.FILE
        srv.plan_request.domain = (get_package_share_directory('upf4ros2_demo')
                                        + str(self._domain.value))
        srv.plan_request.problem = (get_package_share_directory('upf4ros2_demo')
                                        + str(self._problem.value))
        self._plan_pddl_one_shot_client_srv.wait_for_service()
        self.future = self._plan_pddl_one_shot_client_srv.call_async(srv)
        rclpy.spin_until_future_complete(self, self.future)

        plan_result = self.future.result().plan_result
        # self.get_logger().info('Planned Steps are:' + str(plan_result.plan.actions))
        
        for action in plan_result.plan.actions:

            params = [x.symbol_atom[0] for x in action.parameters]
            self.get_logger().info(action.action_name+"("+", ".join(params)+")")
            client = self.create_client(CallAction, 'move')
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')

            req = CallAction.Request()
            req.action_name = action.action_name
            req.parameters = action.parameters

            self.res = client.call_async(req)
            rclpy.spin_until_future_complete(self, self.res)

            if self.res.result().result:
                self.get_logger().info('Action completed')
            else:
                self.get_logger().info('Action cancelled!')
        self.get_logger().info('Plan completed!')


        

def main(args=None):
    rclpy.init(args=args)
    upf4ros2_demo_node = UPF4ROS2DemoNode()

    upf4ros2_demo_node.get_plan_srv()
    rclpy.spin(upf4ros2_demo_node)

    upf4ros2_demo_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

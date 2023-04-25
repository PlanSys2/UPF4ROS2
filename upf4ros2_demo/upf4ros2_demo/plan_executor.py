import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from unified_planning import model
from ament_index_python.packages import get_package_share_directory

from upf4ros2.ros2_interface_reader import ROS2InterfaceReader
from upf4ros2.ros2_interface_writer import ROS2InterfaceWriter
from unified_planning import model
from unified_planning import shortcuts
from upf_msgs import msg as msgs
from upf4ros2_demo_msgs.srv import CallAction

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
    SetInitialValue
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
        self._new_problem = self.create_client(
            NewProblem, 'upf4ros2/srv/new_problem')
        self._add_fluent = self.create_client(
            AddFluent, 'upf4ros2/srv/add_fluent')
        self._add_action = self.create_client(
            AddAction, 'upf4ros2/srv/add_action')
        self._add_object = self.create_client(
            AddObject, 'upf4ros2/srv/add_object')
        self._set_initial_value = self.create_client(
            SetInitialValue, 'upf4ros2/srv/set_initial_value')
        self._add_goal = self.create_client(
            AddGoal, 'upf4ros2/srv/add_goal')
        self._plan_pddl_one_shot_client_srv = self.create_client(
            PDDLPlanOneShotSrv, 'upf4ros2/srv/planOneShotPDDL')  
    

    def new_problem(self, problem_name):
        srv = NewProblem.Request()
        srv.problem_name = problem_name

        self._new_problem.wait_for_service()
        self.future = self._new_problem.call_async(srv)
        rclpy.spin_until_future_complete(self, self.future)

        self._problem_name = problem_name
        self.get_logger().info(f'Create the problem with name: {srv.problem_name}')
        

    def get_problem(self):
        srv = GetProblem.Request()
        srv.problem_name = self._problem_name

        self._get_problem.wait_for_service()
        self.future = self._get_problem.call_async(srv)
        rclpy.spin_until_future_complete(self, self.future)
        # problem = self._ros2_interface_reader.convert(self.future.result().problem)
        problem = self.future.result().problem
        return problem

    def add_fluent(self, problem, fluent_name, user_type):
        fluent = model.Fluent(fluent_name, shortcuts.BoolType(), object=user_type)
        srv = AddFluent.Request()
        srv.problem_name = self._problem_name
        srv.fluent = self._ros2_interface_writer.convert(fluent, problem)

        item = msgs.ExpressionItem()
        item.atom.append(msgs.Atom())
        item.atom[0].boolean_atom.append(False)
        item.type = 'up:bool'
        item.kind = msgs.ExpressionItem.CONSTANT
        value = msgs.Expression()
        value.expressions.append(item)
        value.level.append(0)

        srv.default_value = value

        self._add_fluent.wait_for_service()
        self.future = self._add_fluent.call_async(srv)
        rclpy.spin_until_future_complete(self, self.future)

        self.get_logger().info(f'Add fluent: {fluent_name}')
        return fluent


    def add_object(self, object_name, user_type):
        upf_object = model.Object(object_name, user_type)
        srv = AddObject.Request()
        srv.problem_name = self._problem_name
        srv.object = self._ros2_interface_writer.convert(upf_object)

        self._add_object.wait_for_service()
        self.future = self._add_object.call_async(srv)
        rclpy.spin_until_future_complete(self, self.future)

        self.get_logger().info(f'Add Object: {object_name}')

        return upf_object

    def set_initial_value(self, fluent, object, value_fluent):
        srv = SetInitialValue.Request()
        srv.problem_name = self._problem_name
        srv.expression = self._ros2_interface_writer.convert(fluent(object))

        item = msgs.ExpressionItem()
        item.atom.append(msgs.Atom())
        item.atom[0].boolean_atom.append(value_fluent)
        item.type = 'up:bool'
        item.kind = msgs.ExpressionItem.CONSTANT
        value = msgs.Expression()
        value.expressions.append(item)
        value.level.append(0)

        srv.value = value

        self._set_initial_value.wait_for_service()
        self.future = self._set_initial_value.call_async(srv)
        rclpy.spin_until_future_complete(self, self.future)

        self.get_logger().info(f'Set {fluent.name}({object.name}) with value: {value_fluent}')
    
    def add_action(self, action):
        srv = AddAction.Request()
        srv.problem_name = self._problem_name
        srv.action = self._ros2_interface_writer.convert(action)

        self._add_action.wait_for_service()
        self.future = self._add_action.call_async(srv)
        rclpy.spin_until_future_complete(self, self.future)

        self.get_logger().info(f'Add action: {action.name}')

    def add_goal(self, goal):
        srv = AddGoal.Request()
        srv.problem_name = self._problem_name
        upf_goal = msgs.Goal()
        upf_goal.goal = self._ros2_interface_writer.convert(goal)
        srv.goal.append(upf_goal)

        self._add_goal.wait_for_service()
        self.future = self._add_goal.call_async(srv)
        rclpy.spin_until_future_complete(self, self.future)

        self.get_logger().info(f'Set new goal!')

    def get_plan_action(self):
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


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        for action in feedback.plan_result.plan.actions:

            params = [x.symbol_atom[0] for x in action.parameters]
            self.get_logger().info(action.action_name+"("+params[0]+", "+params[1]+")")

            client = self.create_client(CallAction, action.action_name)
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')

            req = CallAction.Request()
            req.action_name = action.action_name
            req.parameters = action.parameters

            self.res = client.call_async(req)
            rclpy.spin_until_future_complete(self, self.res)

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

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from unified_planning import model

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

from upf_msgs.srv import PlanOneShot as PlanOneShotSrv


class UPF4ROS2DemoNode(Node):

    def __init__(self):
        super().__init__('upf4ros2_demo_navigate')

        self._problem_name = ''
        self._plan_result = {}

        self._ros2_interface_writer = ROS2InterfaceWriter()
        self._ros2_interface_reader = ROS2InterfaceReader()

        self._plan_one_shot_client = ActionClient(
            self,
            PlanOneShot,
            'upf4ros2/planOneShot')

        self._plan_pddl_one_shot_client = ActionClient(
            self,
            PDDLPlanOneShot,
            'upf4ros2/planOneShotPDDL')

        self._get_problem = self.create_client(
            GetProblem, 'upf4ros2/get_problem')
        self._new_problem = self.create_client(
            NewProblem, 'upf4ros2/new_problem')
        self._add_fluent = self.create_client(
            AddFluent, 'upf4ros2/add_fluent')
        self._add_action = self.create_client(
            AddAction, 'upf4ros2/add_action')
        self._add_object = self.create_client(
            AddObject, 'upf4ros2/add_object')
        self._set_initial_value = self.create_client(
            SetInitialValue, 'upf4ros2/set_initial_value')
        self._add_goal = self.create_client(
            AddGoal, 'upf4ros2/add_goal')
        self._plan_one_shot_client_srv = self.create_client(
            PlanOneShotSrv, 'upf4ros2/srv/planOneShot')

    def new_problem(self, problem_name):
        srv = NewProblem.Request()
        srv.problem_name = problem_name

        self._new_problem.wait_for_service()
        self.future = self._new_problem.call_async(srv)
        rclpy.spin_until_future_complete(self, self.future)

        self._problem_name = problem_name
        self.get_logger().info(
            f'Create the problem with name: {srv.problem_name}')

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
        fluent = model.Fluent(
            fluent_name,
            shortcuts.BoolType(),
            object=user_type)
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

        self.get_logger().info(
            f'Set {fluent.name}({object.name}) with value: {value_fluent}')

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
        self.get_logger().info('Planning...')
        problem = self.get_problem()
        goal_msg = PlanOneShot.Goal()
        goal_msg.plan_request.problem = self._ros2_interface_writer.convert(
            problem)

        self._plan_one_shot_client.wait_for_server()
        self.future = self._plan_one_shot_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self.future.add_done_callback(self.goal_response_callback)

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
            self.get_logger().info(
                action.action_name + "(" + params[0] + ", " + params[1] + ")")

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
        srv = PlanOneShotSrv.Request()
        srv.problem = problem

        self._plan_one_shot_client_srv.wait_for_service()

        self.future = self._plan_one_shot_client_srv.call_async(srv)
        rclpy.spin_until_future_complete(self, self.future)

        plan_result = self.future.result().plan_result
        for action in plan_result.plan.actions:

            params = [x.symbol_atom[0] for x in action.parameters]
            self.get_logger().info(action.action_name + "(" + ", ".join(params) + ")")
            client = self.create_client(CallAction, action.action_name)
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


def main(args=None):
    rclpy.init(args=args)

    # test is the name of the problem for the demo
    upf4ros2_demo_node = UPF4ROS2DemoNode()

    upf4ros2_demo_node.new_problem('test')
    problem = upf4ros2_demo_node._ros2_interface_reader.convert(
        upf4ros2_demo_node.get_problem())

    # usertype is the type of the fluent's object
    # usertype can be 'up:bool', 'up:integer', 'up:integer[]', 'up:real',
    # 'up:real[]', shortcuts.UserType('name')
    location = shortcuts.UserType('location')

    robot_at = upf4ros2_demo_node.add_fluent(problem, 'robot_at', location)

    l1 = upf4ros2_demo_node.add_object('livingroom', location)
    l2 = upf4ros2_demo_node.add_object('entrance', location)

    upf4ros2_demo_node.set_initial_value(robot_at, l1, True)
    upf4ros2_demo_node.set_initial_value(robot_at, l2, False)

    # Define navigation action (InstantaneousAction or DurativeAction)
    move = model.InstantaneousAction('move', l_from=location, l_to=location)
    # move = model.DurativeAction('move', l_from=location, l_to=location)
    # If DurativeAction
    # Set the action's duration (set_closed_duration_interval, set_open_duration_interval, set_fixed_duration, set_left_open_duration_interval or set_right_open_duration_interval)
    # move.set_closed_duration_interval(0, 10)

    l_from = move.parameter('l_from')
    l_to = move.parameter('l_to')

    move.add_precondition(robot_at(l_from))
    # move.add_condition(model.StartTiming(), robot_at(l_from))
    move.add_effect(robot_at(l_from), False)
    move.add_effect(robot_at(l_to), True)

    upf4ros2_demo_node.add_action(move)

    upf4ros2_demo_node.add_goal(robot_at(l2))

    upf4ros2_demo_node.get_plan_srv()

    problem_updated = upf4ros2_demo_node._ros2_interface_reader.convert(
        upf4ros2_demo_node.get_problem())
    upf4ros2_demo_node.get_logger().info(f'{problem_updated}')

    rclpy.spin(upf4ros2_demo_node)

    upf4ros2_demo_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

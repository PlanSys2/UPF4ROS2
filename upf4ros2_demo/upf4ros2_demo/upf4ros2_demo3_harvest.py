import rclpy
from rclpy.action import ActionClient
# from rclpy.node import Node
from simple_node import Node

from unified_planning import model

from upf4ros2.ros2_interface_reader import ROS2InterfaceReader
from upf4ros2.ros2_interface_writer import ROS2InterfaceWriter
from unified_planning import model
from unified_planning import shortcuts
from unified_planning.shortcuts import Not
from upf_msgs import msg as msgs
from upf4ros2_demo_msgs.srv import CallAction

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
        super().__init__('upf4ros2_demo3_harvest')

        self._problem_name = ''
        self._plan_result = {}

        self._ros2_interface_writer = ROS2InterfaceWriter()
        self._ros2_interface_reader = ROS2InterfaceReader()

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
            AddGoal, 'upf4ros2/add_goal')
        self._plan_one_shot_client_srv = self.create_client(
            PlanOneShotSrv, 'upf4ros2/srv/planOneShot')
        

    def new_problem(self, problem_name):
        srv = NewProblem.Request()
        srv.problem_name = problem_name

        self._new_problem.wait_for_service()
        self.future = self._new_problem.call(srv)

        self._problem_name = problem_name
        self.get_logger().info(
            f'Create the problem with name: {srv.problem_name}')

    def get_problem(self):
        srv = GetProblem.Request()
        srv.problem_name = self._problem_name

        self._get_problem.wait_for_service()
        self.res = self._get_problem.call(srv)
        # problem = self._ros2_interface_reader.convert(self.future.result().problem)
        problem = self.res.problem
        return problem

    def add_fluent(self, problem, fluent_name, user_types):
        list_parameters = [model.Parameter(f"{t.name}_{i}", t) for i, t in enumerate(user_types)]
    
        fluent = model.Fluent(fluent_name, shortcuts.BoolType(), list_parameters)
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
        self.future = self._add_fluent.call(srv)

        self.get_logger().info(f'Add fluent: {fluent_name}')
        return fluent

    def add_object(self, object_name, user_type):
        upf_object = model.Object(object_name, user_type)
        srv = AddObject.Request()
        srv.problem_name = self._problem_name
        srv.object = self._ros2_interface_writer.convert(upf_object)

        self._add_object.wait_for_service()
        self.future = self._add_object.call(srv)

        self.get_logger().info(f'Add Object: {object_name}')

        return upf_object

    def set_initial_value(self, fluent, objects, value_fluent):
        srv = SetInitialValue.Request()
        srv.problem_name = self._problem_name
        srv.expression = self._ros2_interface_writer.convert(fluent(*objects))

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
        self.future = self._set_initial_value.call(srv)

        self.get_logger().info(
            f'Set {fluent.name}({(object.name for object in objects)}) with value: {value_fluent}')

    def add_action(self, action):
        srv = AddAction.Request()
        srv.problem_name = self._problem_name
        srv.action = self._ros2_interface_writer.convert(action)

        self._add_action.wait_for_service()
        self.future = self._add_action.call(srv)

        self.get_logger().info(f'Add action: {action.name}')

    def add_goal(self, goal):
        srv = AddGoal.Request()
        srv.problem_name = self._problem_name
        upf_goal = msgs.Goal()
        upf_goal.goal = self._ros2_interface_writer.convert(goal)
        srv.goal.append(upf_goal)

        self._add_goal.wait_for_service()
        self.future = self._add_goal.call(srv)

        self.get_logger().info(f'Set new goal!')

    def __cancel_callback(self, action):
        # return to old wp
        if action.action_name == 'move':
            client = self.create_client(CallAction, action.action_name)
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')

            req = CallAction.Request()
            req.action_name = action.action_name
            req.parameters = action.parameters[::-1]
            self.res = client.call(req)
            if self.res.result:
                self.get_logger().info('Return to origin wp')
            else:
                self.get_logger().info('Can not return to the origin wp!')

    def get_plan_srv(self):

        problem = self.get_problem()

        self.get_logger().info('Planning...')
        srv = PlanOneShotSrv.Request()
        srv.problem = problem

        self._plan_one_shot_client_srv.wait_for_service()

        self.res = self._plan_one_shot_client_srv.call(srv)

        plan_result = self.res.plan_result

        for action in plan_result.plan.actions:

            params = [x.symbol_atom[0] for x in action.parameters]
            self.get_logger().info(action.action_name + "(" + ", ".join(params) + ")")

        for action in plan_result.plan.actions:

            client = self.create_client(CallAction, action.action_name)
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            req = CallAction.Request()
            req.action_name = action.action_name
            req.parameters = action.parameters

            self.res = client.call(req)

            if self.res.result:
                self.get_logger().info('Action completed')
            else:
                self.get_logger().info('Action cancelled!')
                self.__cancel_callback(action)


def main(args=None):
    rclpy.init(args=args)

    # test is the name of the problem for the demo
    upf4ros2_demo_node = UPF4ROS2DemoNode()

    upf4ros2_demo_node.new_problem('harvest')
    problem = upf4ros2_demo_node._ros2_interface_reader.convert(
        upf4ros2_demo_node.get_problem())


    # usertype is the type of the fluent's object
    # usertype can be 'up:bool', 'up:integer', 'up:integer[]', 'up:real',
    # 'up:real[]', shortcuts.UserType('name')
    location = shortcuts.UserType('location')
    crop = shortcuts.UserType('crop')

    robot_at = upf4ros2_demo_node.add_fluent(problem, 'robot_at', [location])
    wp_checked = upf4ros2_demo_node.add_fluent(problem, 'wp_checked', [location])
    crop_at = upf4ros2_demo_node.add_fluent(problem, 'crop_at', [crop, location])
    is_connected = upf4ros2_demo_node.add_fluent(problem, 'is_connected', [location, location])
    is_mature = upf4ros2_demo_node.add_fluent(problem, 'is_mature', [crop])
    collected = upf4ros2_demo_node.add_fluent(problem, 'collected', [crop])

    base = upf4ros2_demo_node.add_object('base', location)
    wp1 = upf4ros2_demo_node.add_object('wp1', location)
    wp2 = upf4ros2_demo_node.add_object('wp2', location)
    wp3 = upf4ros2_demo_node.add_object('wp3', location)
    tomato1 = upf4ros2_demo_node.add_object('tomato1', crop)
    tomato2 = upf4ros2_demo_node.add_object('tomato2', crop)
    tomato3 = upf4ros2_demo_node.add_object('tomato3', crop)


    upf4ros2_demo_node.set_initial_value(robot_at, [base], True)
    upf4ros2_demo_node.set_initial_value(is_connected, [base, wp1], True)
    upf4ros2_demo_node.set_initial_value(is_connected, [wp1, wp2], True)
    upf4ros2_demo_node.set_initial_value(is_connected, [wp2, wp3], True)
    upf4ros2_demo_node.set_initial_value(is_connected, [wp3, base], True)
    upf4ros2_demo_node.set_initial_value(is_connected, [wp2, base], True)
    upf4ros2_demo_node.set_initial_value(crop_at, [tomato1, wp1], True)
    upf4ros2_demo_node.set_initial_value(crop_at, [tomato2, wp2], True)
    upf4ros2_demo_node.set_initial_value(crop_at, [tomato3, wp3], True)
    upf4ros2_demo_node.set_initial_value(is_mature, [tomato3], False)
    upf4ros2_demo_node.set_initial_value(is_mature, [tomato2], True)

    # Define navigation action (InstantaneousAction or DurativeAction)
    move = model.InstantaneousAction('move', l_from=location, l_to=location)
    # move = model.DurativeAction('move', l_from=location, l_to=location)
    # If DurativeAction
    # Set the action's duration (set_closed_duration_interval, set_open_duration_interval, set_fixed_duration, set_left_open_duration_interval or set_right_open_duration_interval)
    # move.set_closed_duration_interval(0, 10)

    l_from = move.parameter('l_from')
    l_to = move.parameter('l_to')

    move.add_precondition(robot_at(l_from))
    move.add_precondition(is_connected(l_from, l_to))
    # move.add_condition(model.StartTiming(), robot_at(l_from))
    move.add_effect(robot_at(l_from), False)
    move.add_effect(robot_at(l_to), True)

    # ------------------------------------- #
    # Define check wp action
    check_wp = model.InstantaneousAction('check_wp', wp=location)
    wp = check_wp.parameter('wp')
    check_wp.add_precondition(robot_at(wp))
    check_wp.add_effect(wp_checked(wp), True)

    # ------------------------------------- #
    # Collect action
    collect = model.InstantaneousAction('collect', c=crop, l_at=location)
    c = collect.parameter('c')
    l_at = collect.parameter('l_at')
    collect.add_precondition(robot_at(l_at))
    collect.add_precondition(crop_at(c,l_at))
    collect.add_precondition(is_mature(c))
    collect.add_precondition(Not(collected(c)))
    collect.add_effect(collected(c), True)

    # ------------------------------------- #
    # Unload action
    # unload = model.InstantaneousAction('unload', c=crop, l_to=location)
    # c_c = unload.parameter('c')
    # l = unload.parameter('l_to')
    # unload.add_precondition(robot_at(l))
    # unload.add_precondition(collected(c_c))
    # unload.add_effect(collected(c_c), False)

    upf4ros2_demo_node.add_action(move)
    upf4ros2_demo_node.add_action(check_wp)
    upf4ros2_demo_node.add_action(collect)


    upf4ros2_demo_node.add_goal(robot_at(base))
    upf4ros2_demo_node.add_goal(collected(tomato2))


    problem_old_upf = upf4ros2_demo_node.get_problem()
    problem_old = upf4ros2_demo_node._ros2_interface_reader.convert(
        problem_old_upf)
    upf4ros2_demo_node.get_logger().info(f'{problem_old}')

    upf4ros2_demo_node.get_plan_srv()

    problem_updated = upf4ros2_demo_node._ros2_interface_reader.convert(
        upf4ros2_demo_node.get_problem())
    upf4ros2_demo_node.get_logger().info(f'{problem_updated}')

    upf4ros2_demo_node.join_spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

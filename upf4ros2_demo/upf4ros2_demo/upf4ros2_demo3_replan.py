import time

import rclpy
from rclpy.node import Node
from unified_planning import model, shortcuts

from upf4ros2.ros2_interface_reader import ROS2InterfaceReader
from upf4ros2.ros2_interface_writer import ROS2InterfaceWriter
from upf4ros2_demo_msgs.srv import CallAction
from upf_msgs import msg as msgs
from upf_msgs.srv import (
    AddAction,
    AddFluent,
    AddGoal,
    AddObject,
    GetProblem,
    NewProblem,
    PlanOneShot as PlanOneShotSrv,
    SetInitialValue,
)


class UPF4ROS2Demo3ReplanNode(Node):

    def __init__(self):
        super().__init__('upf4ros2_demo3_replan')

        self.declare_parameter('problem_name', 'replan_demo')
        self.declare_parameter('execute_actions', True)
        self.declare_parameter('simulate_block_first_move', True)
        self.declare_parameter('max_replans', 3)

        self._problem_name = self.get_parameter('problem_name').value
        self._execute_actions = bool(self.get_parameter('execute_actions').value)
        self._simulate_block_first_move = bool(
            self.get_parameter('simulate_block_first_move').value)
        self._max_replans = int(self.get_parameter('max_replans').value)

        self._writer = ROS2InterfaceWriter()
        self._reader = ROS2InterfaceReader()
        self._location_type = shortcuts.UserType('location')
        self._objects = {}
        self._current_location = 'livingroom'
        self._checked = set()
        self._blocked_once = False
        self._replans = 0

        self._get_problem_client = self.create_client(GetProblem, 'upf4ros2/srv/get_problem')
        self._new_problem_client = self.create_client(NewProblem, 'upf4ros2/srv/new_problem')
        self._add_fluent_client = self.create_client(AddFluent, 'upf4ros2/srv/add_fluent')
        self._add_action_client = self.create_client(AddAction, 'upf4ros2/srv/add_action')
        self._add_object_client = self.create_client(AddObject, 'upf4ros2/srv/add_object')
        self._set_initial_value_client = self.create_client(
            SetInitialValue, 'upf4ros2/srv/set_initial_value')
        self._add_goal_client = self.create_client(AddGoal, 'upf4ros2/add_goal')
        self._plan_one_shot_client = self.create_client(
            PlanOneShotSrv, 'upf4ros2/srv/planOneShot')

    def _call(self, client, request, timeout_s=30.0):
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError(f'Service {client.srv_name} is not available')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'Service {client.srv_name} did not respond')
        response = future.result()
        if hasattr(response, 'success') and not response.success:
            message = getattr(response, 'message', '')
            raise RuntimeError(f'Service {client.srv_name} failed: {message}')
        return response

    def _bool_expression(self, value):
        item = msgs.ExpressionItem()
        item.atom.append(msgs.Atom())
        item.atom[0].boolean_atom.append(value)
        item.type = 'up:bool'
        item.kind = msgs.ExpressionItem.CONSTANT
        expression = msgs.Expression()
        expression.expressions.append(item)
        expression.level.append(0)
        return expression

    def _get_upf_problem_msg(self):
        request = GetProblem.Request()
        request.problem_name = self._problem_name
        return self._call(self._get_problem_client, request).problem

    def _add_fluent_to_problem(self, problem, fluent_name, *params):
        fluent = model.Fluent(fluent_name, shortcuts.BoolType(), **dict(params))
        request = AddFluent.Request()
        request.problem_name = self._problem_name
        request.fluent = self._writer.convert(fluent, problem)
        request.default_value = self._bool_expression(False)
        self._call(self._add_fluent_client, request)
        self.get_logger().info(f'Added fluent {fluent_name}')
        return fluent

    def _add_object_to_problem(self, name):
        upf_object = model.Object(name, self._location_type)
        request = AddObject.Request()
        request.problem_name = self._problem_name
        request.object = self._writer.convert(upf_object)
        self._call(self._add_object_client, request)
        self._objects[name] = upf_object
        return upf_object

    def _set_initial_value(self, fluent, objects, value):
        request = SetInitialValue.Request()
        request.problem_name = self._problem_name
        request.expression = self._writer.convert(fluent(*objects))
        request.value = self._bool_expression(value)
        self._call(self._set_initial_value_client, request)

    def _add_action_to_problem(self, action):
        request = AddAction.Request()
        request.problem_name = self._problem_name
        request.action = self._writer.convert(action)
        self._call(self._add_action_client, request)
        self.get_logger().info(f'Added action {action.name}')

    def _add_goal_to_problem(self, goal):
        request = AddGoal.Request()
        request.problem_name = self._problem_name
        goal_msg = msgs.Goal()
        goal_msg.goal = self._writer.convert(goal)
        request.goal.append(goal_msg)
        self._call(self._add_goal_client, request)

    def _create_problem(self):
        request = NewProblem.Request()
        request.problem_name = self._problem_name
        self._call(self._new_problem_client, request)

        problem = self._reader.convert(self._get_upf_problem_msg())

        self.robot_at = self._add_fluent_to_problem(problem, 'robot_at', ('object', self._location_type))
        self.wp_checked = self._add_fluent_to_problem(problem, 'wp_checked', ('object', self._location_type))
        self.connected = self._add_fluent_to_problem(
            problem, 'connected', ('l_from', self._location_type), ('l_to', self._location_type))

        for name in ['livingroom', 'entrance', 'kitchen', 'bedroom', 'room', 'gym']:
            self._add_object_to_problem(name)

        for name, obj in self._objects.items():
            self._set_initial_value(self.robot_at, [obj], name == self._current_location)
            self._set_initial_value(self.wp_checked, [obj], False)

        self._edges = [
            ('livingroom', 'entrance'),
            ('entrance', 'livingroom'),
            ('livingroom', 'kitchen'),
            ('kitchen', 'livingroom'),
            ('entrance', 'kitchen'),
            ('kitchen', 'entrance'),
            ('kitchen', 'bedroom'),
            ('bedroom', 'kitchen'),
            ('bedroom', 'room'),
            ('room', 'bedroom'),
            ('room', 'gym'),
            ('gym', 'room'),
            ('kitchen', 'gym'),
            ('gym', 'kitchen'),
        ]
        for edge in self._edges:
            self._set_connected(edge[0], edge[1], True)

        move = model.InstantaneousAction(
            'move', l_from=self._location_type, l_to=self._location_type)
        l_from = move.parameter('l_from')
        l_to = move.parameter('l_to')
        move.add_precondition(self.robot_at(l_from))
        move.add_precondition(self.connected(l_from, l_to))
        move.add_effect(self.robot_at(l_from), False)
        move.add_effect(self.robot_at(l_to), True)

        check_wp = model.InstantaneousAction('check_wp', wp=self._location_type)
        wp = check_wp.parameter('wp')
        check_wp.add_precondition(self.robot_at(wp))
        check_wp.add_effect(self.wp_checked(wp), True)

        self._add_action_to_problem(move)
        self._add_action_to_problem(check_wp)

        for goal in ['entrance', 'gym', 'kitchen']:
            self._add_goal_to_problem(self.wp_checked(self._objects[goal]))

    def _set_connected(self, source, target, value):
        self._set_initial_value(
            self.connected, [self._objects[source], self._objects[target]], value)

    def _set_robot_location(self, location):
        old_location = self._current_location
        self._set_initial_value(self.robot_at, [self._objects[old_location]], False)
        self._set_initial_value(self.robot_at, [self._objects[location]], True)
        self._current_location = location

    def _set_checked(self, location):
        self._set_initial_value(self.wp_checked, [self._objects[location]], True)
        self._checked.add(location)

    def _get_plan(self, reason):
        problem = self._get_upf_problem_msg()
        request = PlanOneShotSrv.Request()
        request.problem = problem
        self.get_logger().info(f'Planning request ({reason})')
        start = time.monotonic()
        response = self._call(self._plan_one_shot_client, request)
        elapsed = time.monotonic() - start
        plan_result = response.plan_result
        actions = list(plan_result.plan.actions)
        if not response.success or not actions:
            raise RuntimeError(f'No plan found during {reason}: {response.message}')
        self.get_logger().info(
            f'Planning response ({reason}): {len(actions)} actions in {elapsed:.4f} s')
        for action in actions:
            self.get_logger().info('  ' + self._action_to_string(action))
        return actions

    def _action_to_string(self, action):
        params = [param.symbol_atom[0] for param in action.parameters]
        return action.action_name + '(' + ', '.join(params) + ')'

    def _action_params(self, action):
        return [param.symbol_atom[0] for param in action.parameters]

    def _call_action_service(self, action):
        client = self.create_client(CallAction, action.action_name)
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f'Action service {action.action_name} is not available')
        request = CallAction.Request()
        request.action_name = action.action_name
        request.parameters = action.parameters
        response = self._call(client, request, timeout_s=300.0)
        return bool(response.result)

    def _execute_or_simulate(self, action):
        params = self._action_params(action)

        if action.action_name == 'move':
            source, target = params
            if self._simulate_block_first_move and not self._blocked_once:
                self._blocked_once = True
                self.get_logger().warn(
                    f'Simulating blocked connection {source} -> {target}; replanning required')
                self._set_connected(source, target, False)
                return False

            if self._execute_actions:
                success = self._call_action_service(action)
                if not success:
                    self.get_logger().warn(
                        f'Move {source} -> {target} failed; disabling connection and replanning')
                    self._set_connected(source, target, False)
                    return False

            self._set_robot_location(target)
            return True

        if action.action_name == 'check_wp':
            waypoint = params[0]
            if waypoint != self._current_location:
                self.get_logger().warn(
                    f'Cannot check {waypoint}: robot is at {self._current_location}')
                return False
            if self._execute_actions:
                success = self._call_action_service(action)
                if not success:
                    return False
            self._set_checked(waypoint)
            return True

        self.get_logger().warn(f'Unknown action {action.action_name}')
        return False

    def run(self):
        self._create_problem()
        plan = self._get_plan('initial')

        while rclpy.ok():
            failed = False
            for action in plan:
                self.get_logger().info(f'Executing {self._action_to_string(action)}')
                if not self._execute_or_simulate(action):
                    failed = True
                    break

            if {'entrance', 'gym', 'kitchen'}.issubset(self._checked):
                self.get_logger().info(
                    f'Mission completed with {self._replans} replanning events')
                return True

            if not failed:
                self.get_logger().error('Plan ended before all goals were achieved')
                return False

            if self._replans >= self._max_replans:
                self.get_logger().error('Maximum replans reached; mission failed')
                return False

            self._replans += 1
            self.get_logger().warn(f'Replanning triggered ({self._replans}/{self._max_replans})')
            plan = self._get_plan(f'replan_{self._replans}')


def main(args=None):
    rclpy.init(args=args)
    node = UPF4ROS2Demo3ReplanNode()
    try:
        success = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())

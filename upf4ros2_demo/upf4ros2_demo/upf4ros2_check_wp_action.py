from typing import List
import rclpy

from geometry_msgs.msg import Pose
from nav2_msgs.action import NavigateToPose
from upf4ros2_demo_msgs.srv import CallAction
from upf4ros2.ros2_interface_reader import ROS2InterfaceReader
from upf4ros2.ros2_interface_writer import ROS2InterfaceWriter
from unified_planning import model
from unified_planning import shortcuts
from upf_msgs import msg as msgs
from std_msgs.msg import String
from upf4ros2_demo_msgs.srv import CallAction

from upf_msgs.srv import (
    SetInitialValue,
    GetProblem
)

from simple_node import Node


class CheckWpAction(Node):

    def __init__(self):
        super().__init__('upf4ros2_check_wp_action')

        self._problem_name = 'test'

        self._userType = shortcuts.UserType('location')
        self._fluent = model.Fluent(
            "wp_checked",
            shortcuts.BoolType(),
            object=self._userType)
        self._fluent_robot_at = model.Fluent(
            "robot_at", shortcuts.BoolType(), object=self._userType)

        self._ros2_interface_writer = ROS2InterfaceWriter()
        self._ros2_interface_reader = ROS2InterfaceReader()

        self._get_problem = self.create_client(
            GetProblem, 'upf4ros2/get_problem')
        self._set_initial_value = self.create_client(
            SetInitialValue, 'upf4ros2/set_initial_value')

        self.create_service(
            CallAction, 'check_wp', self.__execute_callback)

    def get_problem(self):
        """ get actual state of the problem
        Args:

        """
        srv = GetProblem.Request()
        srv.problem_name = self._problem_name

        self._get_problem.wait_for_service()
        self.res = self._get_problem.call(srv)
        # problem = self._ros2_interface_reader.convert(self.future.result().problem)
        problem = self.res.problem
        return problem

    def set_initial_value(self, fluent, object, value_fluent):
        """ set initial value to the fluent
        Args:
            fluent (up.model.Fluent): fluent
            object (up.model.Object): fluent's object
            value_fluent (bool): new value
        """
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
        self.future = self._set_initial_value.call(srv)

        self.get_logger().info(
            f'Set {fluent.name}({object.name}) with value: {value_fluent}')

    def __execute_callback(self, request, response):
        """ srv callback to call the NavigateToPose action
        Args:
            request (CallAction.Request): request with the action's name and parameters
            response (CallAction.Response): response with the result of the action
        Returns:
            CallAction.Response: response with the result of the action
        """
        problem = self._ros2_interface_reader.convert(self.get_problem())

        wp = request.parameters[0].symbol_atom[0]
        l1 = model.Object(wp, self._userType)

        if str(problem.initial_values[self._fluent_robot_at(l1)]) == 'true':

            self.get_logger().info("Starting action " + request.action_name)
            self.get_logger().info("Waypoint " + str(wp) + " checked.")

            self.set_initial_value(self._fluent, l1, True)
            response.result = True
        else:
            self.get_logger().info("Cannot check wp because the wp was not accesible.")
            response.result = False

        return response


def main(args=None):
    rclpy.init(args=args)

    check_wp_action = CheckWpAction()

    check_wp_action.join_spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

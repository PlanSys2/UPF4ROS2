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


class CollectAction(Node):

    def __init__(self):
        super().__init__('upf4ros2_collect_action')

        self._problem_name = 'test'

        self._userType_location = shortcuts.UserType('location')
        self._userType_crop = shortcuts.UserType('crop')
        self._fluent_collected = model.Fluent(
            "collected",
            shortcuts.BoolType(),
            [model.Parameter('c', self._userType_crop)])
        
        self._fluent_is_mature = model.Fluent(
            "is_mature", shortcuts.BoolType(), [model.Parameter('c', self._userType_crop)])

        self._ros2_interface_writer = ROS2InterfaceWriter()
        self._ros2_interface_reader = ROS2InterfaceReader()

        self._get_problem = self.create_client(
            GetProblem, 'upf4ros2/srv/get_problem')
        self._set_initial_value = self.create_client(
            SetInitialValue, 'upf4ros2/srv/set_initial_value')

        self.create_service(
            CallAction, 'collect', self.__execute_callback)

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
        #self.get_logger().info("entra en collect")
        # problem = self._ros2_interface_reader.convert(self.get_problem())
        #self.get_logger().info(f'{request}')

        crop = request.parameters[0].symbol_atom[0]
        c = model.Object(crop, self._userType_crop)

        # self.get_logger().info(f'initial values: {problem.initial_values}')
        # self.get_logger().info(f'is_mature: {problem.initial_values[self._fluent_is_mature(c)]}')

        # if str(problem.initial_values[self._fluent_is_mature(c)]) == 'true':

        self.get_logger().info("Starting action " + request.action_name)
        self.get_logger().info(str(crop) + " collected.")

        self.set_initial_value(self._fluent_collected, c, True)
        response.result = True
        # else:
        #     self.get_logger().info("Cannot collect "+str(crop)+" because the is not madure.")
        #     response.result = False

        return response


def main(args=None):
    rclpy.init(args=args)

    check_wp_action = CollectAction()

    check_wp_action.join_spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

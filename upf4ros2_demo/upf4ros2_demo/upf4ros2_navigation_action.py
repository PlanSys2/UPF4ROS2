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
from upf4ros2_demo_msgs.srv import CallAction

from upf_msgs.srv import (
    SetInitialValue,
)

from simple_node import Node


class NavigationAction(Node):

    def __init__(self):
        super().__init__('upf4ros2_navigation_action')

        self.__wp_dict = {}

        wps_param_name = "wps"

        # declaring params
        self.declare_parameter(wps_param_name, [''])

        # getting params
        waypoints = self.get_parameter(
            wps_param_name).get_parameter_value().string_array_value

        # load points
        self.load_wps(waypoints)

        self._problem_name = 'test'

        self._userType = shortcuts.UserType('location')
        self._fluent = model.Fluent(
            "robot_at",
            shortcuts.BoolType(),
            object=self._userType)

        self._ros2_interface_writer = ROS2InterfaceWriter()
        self._ros2_interface_reader = ROS2InterfaceReader()

        self.__nav_to_pose_client = self.create_action_client(
            NavigateToPose,
            '/navigate_to_pose')

        self._set_initial_value = self.create_client(
            SetInitialValue, 'upf4ros2/set_initial_value')

        self.create_service(
            CallAction, 'move', self.__execute_callback)

    def load_wps(self, waypoints: List[str]):
        """ load waypoints of list strings into a dictionary of floats
        Args:
            points (List[str]): list of points
        """

        if not waypoints:
            return

        for i in range(0, len(waypoints), 5):
            self.__wp_dict[waypoints[i]] = Pose()
            self.__wp_dict[waypoints[i]].position.x = float(waypoints[i + 1])
            self.__wp_dict[waypoints[i]].position.y = float(waypoints[i + 2])
            self.__wp_dict[waypoints[i]].orientation.z = float(
                waypoints[i + 3])
            self.__wp_dict[waypoints[i]].orientation.w = float(
                waypoints[i + 4])

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
        l1 = model.Object(request.parameters[0].symbol_atom[0], self._userType)
        l2 = model.Object(request.parameters[1].symbol_atom[0], self._userType)

        self.get_logger().info("Starting action " + request.action_name)

        self.get_logger().info("Waiting for 'NavigateToPose' action server")
        while not self.__nav_to_pose_client.wait_for_server():
            self.get_logger().info("'NavigateToPose' action server not available, waiting...")

        wp = request.parameters[1].symbol_atom[0]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose = self.__wp_dict[wp]
        goal_msg.pose.header.frame_id = "map"

        self.get_logger().info('Navigating to goal: ' + str(wp))

        self.__nav_to_pose_client.send_goal(goal_msg)
        self.__nav_to_pose_client.wait_for_result()

        if self.__nav_to_pose_client.is_succeeded():
            self.get_logger().info("Goal to " + str(wp) + " done")
            self.set_initial_value(self._fluent, l1, False)
            self.set_initial_value(self._fluent, l2, True)
            response.result = True
        else:
            self.get_logger().info("Goal to " + str(wp) + " was rejected!")
            response.result = False

        return response


def main(args=None):
    rclpy.init(args=args)

    navigation_action = NavigationAction()

    navigation_action.join_spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

from typing import List
import tempfile
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node

from geometry_msgs.msg import Pose, PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from upf4ros2_demo_interfaces.srv import CallAction


class NavigationAction(Node):

    def __init__(self):
        super().__init__('upf4ros2_navigation_action')

        self.__wp_dict = {}

        wps_param_name = "wps"

        # declaring params
        self.declare_parameter(wps_param_name, [''])

        # getting params
        waypoints = self.get_parameter(wps_param_name).get_parameter_value().string_array_value

        # load points
        self.load_wps(waypoints)
        

        self.srv = self.create_service(CallAction, 'move', self.execute_callback)

        self._nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose')
    
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
        
    def execute_callback(self, request, response):
        self.get_logger().info("Waiting for 'NavigateToPose' action server")
        while not self._nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'NavigateToPose' action server not available, waiting...")


        wp = request.parameters[0]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose = self.__wp_dict[wp]
        goal_msg.pose.header.frame_id = "map"

        self.get_logger().info('Navigating to goal: ' + str(goal_msg.pose.pose.position.x) + ' ' +
                      str(goal_msg.pose.pose.position.y) + '...')
        send_goal_future = self._nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.get_logger().info('Goal to ' + str(goal_msg.pose.pose.position.x) + ' ' +
                           str(goal_msg.pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        self.get_logger().info(f'{self.feedback}')
        return


        

def main(args=None):
    rclpy.init(args=args)

    navigation_action = NavigationAction()

    rclpy.spin(navigation_action)

    navigation_action.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

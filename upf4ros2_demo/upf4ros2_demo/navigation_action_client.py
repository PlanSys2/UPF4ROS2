from typing import List
import rclpy
import json

from ament_index_python.packages import get_package_share_directory
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
        super().__init__('navigation_action_client')

        # declaring params
        self.declare_parameter('lookupTable', '/params/lookupTable.json')
        self._lookupTable = self.get_parameter('lookupTable')
        lookupTablePath = (get_package_share_directory('upf4ros2_demo')
                                        + str(self._lookupTable.value))
    
        self._lookupTable = dict()
        with open(lookupTablePath) as file:
            self._lookupTable = json.load(file)

        # set initial home coordinates; overwrite later
        self._lookupTable['home'] = [0,0,0]
        self._problem_name = 'uav-test'

        self._ros2_interface_writer = ROS2InterfaceWriter()
        self._ros2_interface_reader = ROS2InterfaceReader()
        
        self.__nav_to_pose_client = self.create_action_client(
            NavigateToPose,
            '/navigate_to_pose')
            
        self.__takeoff_client = self.create_action_client(
            NavigateToPose,
            '/takeoff')
            
        self.__landing_client = self.create_action_client(
            NavigateToPose,
            '/landing')

        self.create_service(
            CallAction, 'move', self.__execute_callback)


        
    def __execute_callback(self, request, response):
        """ srv callback to call the NavigateToPose action
        Args:
            request (CallAction.Request): request with the action's name and parameters
            response (CallAction.Response): response with the result of the action
        Returns:
            CallAction.Response: response with the result of the action
        """
        
        # Waiting for connection to all action servers in Offboard Control
        """self.get_logger().info("Waiting for 'Takeoff' action server")
        self.get_logger().info("Waiting for 'NavigateToPose' action server")
        self.get_logger().info("Waiting for 'Landing' action server")
        """
        
        while not self.__nav_to_pose_client.wait_for_server():
            self.get_logger().info("'NavigateToPose' action server not available, waiting...")
        
        while not self.__takeoff_client.wait_for_server():
            self.get_logger().info("'Takeoff' action server not available, waiting...")
            
        
        while not self.__landing_client.wait_for_server():
            self.get_logger().info("'Landing' action server not available, waiting...")
        
        
        
        self.get_logger().info("Starting action " + request.action_name)
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        
        if request.action_name == "fly":
        
            wp = request.parameters[2].symbol_atom[0]
            wp_resolved = self._lookupTable[wp]
            
            self.get_logger().info("Printing Waypoint Coordinates")
            self.get_logger().info(str(wp_resolved))
            
            
            wp_pose = Pose()
            wp_pose.position.x = float(wp_resolved[0])
            wp_pose.position.y = float(wp_resolved[1])
            wp_pose.position.z = float(wp_resolved[2])
            
            goal_msg.pose.pose = wp_pose
            
            
            
            self.get_logger().info("Sending Waypoint Goal Message")
            self.__nav_to_pose_client.send_goal(goal_msg)
            self.__nav_to_pose_client.wait_for_result()

            if self.__nav_to_pose_client.is_succeeded():
                self.get_logger().info("Goal to " + str(wp) + " done")
                response.result = True
            else:
                self.get_logger().info("Goal to " + str(wp) + " was rejected!")
                response.result = False
                
        elif request.action_name == "take_off":
            self.__takeoff_client.send_goal(goal_msg)
            self.__takeoff_client.wait_for_result()

            if self.__takeoff_client.is_succeeded():
                self.get_logger().info("Takeoff Succeded")
                response.result = True
            else:
                self.get_logger().info("Takeoff Failed")
                response.result = False
        elif request.action_name == "land":
            self.__landing_client.send_goal(goal_msg)
            self.__landing_client.wait_for_result()

            if self.__landing_client.is_succeeded():
                self.get_logger().info("Landing Succeded")
                response.result = True
            else:
                self.get_logger().info("Landing Failed")
                response.result = False
        
        return response
        

def main(args=None):
    rclpy.init(args=args)

    navigation_action = NavigationAction()

    navigation_action.join_spin()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

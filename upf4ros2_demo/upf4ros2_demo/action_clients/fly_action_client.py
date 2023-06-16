from rclpy import logging
import json

from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Pose

# TODO: remove duplicate code in action clients
class FlyActionClient():
    def __init__(self, node, feedback_callback, result_callback):
        self.logger = logging.get_logger(__class__.__name__)

        lookupTablePath = (get_package_share_directory('upf4ros2_demo')
                                        + str('/params/lookupTable.json'))
        self._lookupTable = dict()
        with open(lookupTablePath) as file:
            self._lookupTable = json.load(file)

        # set initial home coordinates; overwrite later
        self._lookupTable['home'] = [0,0,0]

        self.__fly_client = ActionClient(node, NavigateToPose, '/navigate_to_pose')

        self._action = None
        self._params = []
        # TODO: add feedback callback
        self.feedback_callback = feedback_callback
        self.result_callback = result_callback

    def send_action_goal(self, actionInstance, params):
        # not working in current state -> blocks without publishing message
        #while not self.__fly_client.wait_for_server():
        #    self.logger.info("'Fly' action server not available, waiting...")
        self.logger.info("Starting action 'Fly'")
        self._action = actionInstance
        self._params = params

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        
        # unpack coordinates from ActionInstance message
        wp = actionInstance.parameters[2].symbol_atom[0]
        wp_resolved = self._lookupTable[wp]
        
        self.logger.info("Printing Waypoint Coordinates")
        self.logger.info(str(wp_resolved))
        
        # construct Pose from unpacked coordinates; Pose is required by the NavigateToPose action
        wp_pose = Pose()
        wp_pose.position.x = float(wp_resolved[0])
        wp_pose.position.y = float(wp_resolved[1])
        wp_pose.position.z = float(wp_resolved[2])
        
        goal_msg.pose.pose = wp_pose
        self.logger.info("2'Fly'")
        self._send_goal_future1 = self.__fly_client.send_goal_async(goal_msg)
        self._send_goal_future1.add_done_callback(self.goal_response_callback)
        self.logger.info("end 'Fly'")

    def goal_response_callback(self, future):
        self.logger.info("response callback 'Fly'")
        goal_handle = future.result()
        self.logger.info("1 response callback 'Fly'")
        if not goal_handle.accepted:
            self.logger.info('Error! Goal rejected')
            return
        self.logger.info("3 response callback 'Fly'")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        self.logger.info("4 response callback 'Fly'")

    def get_result_callback(self, future):
        self.logger.info("5 'Fly'")
        self.result_callback(self._action, self._params, future.result().result)

    def cancel_action_goal(self):
        None

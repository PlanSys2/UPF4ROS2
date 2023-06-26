from upf4ros2_demo.action_clients.customaction_client import CustomActionClient
from ament_index_python.packages import get_package_share_directory
import json
from geometry_msgs.msg import Pose


class InspectActionClient(CustomActionClient):

    def __init__(self, node, feedback_callback, result_callback):
        super().__init__(node, feedback_callback, result_callback,"/navigate_to_pose")
        lookupTablePath = (get_package_share_directory('upf4ros2_demo')
                                + str('/params/lookupTable.json'))
        self._lookupTable = dict()
        with open(lookupTablePath) as file:
            self._lookupTable = json.load(file)
        # set initial home coordinates; overwrite later
        self._lookupTable['home'] = [0,0,0]
    
    
    def create_goalmsg(self, goal_msg):
        wp = self._action.parameters[1].symbol_atom[0]
        wp_resolved = self._lookupTable[wp]
        self.logger.info("Printing Waypoint Coordinates")
        self.logger.info(str(wp_resolved))
        # construct Pose from unpacked coordinates; Pose is required by the NavigateToPose action
        wp_pose = Pose()
        wp_pose.position.x = float(wp_resolved[0])
        wp_pose.position.y = float(wp_resolved[1])
        wp_pose.position.z = float(wp_resolved[2])
        goal_msg.pose.pose = wp_pose
        
        return goal_msg
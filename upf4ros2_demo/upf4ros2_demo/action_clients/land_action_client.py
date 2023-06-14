from rclpy import logging

from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

# TODO: remove duplicate code in action clients
class LandActionClient():
    def __init__(self, node, feedback_callback, result_callback):
        self.logger = logging.get_logger(__class__.__name__)

        self.__land_client = ActionClient(node, NavigateToPose, '/landing')

        self._action = None
        self._params = []
        # TODO: add feedback callback
        self.feedback_callback = feedback_callback
        self.result_callback = result_callback

    def send_action_goal(self, actionInstance, params):
        while not self.__land_client.wait_for_server():
            self.logger.info("'Land' action server not available, waiting...")
        self.logger.info("Starting action 'Land'")
        self._action = actionInstance
        self._params = params

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        

        self._send_goal_future = self.__land_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.info('Error! Goal rejected')
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.result_callback(self._action, self._params, future.result().result)

    def cancel_action_goal(self):
        None

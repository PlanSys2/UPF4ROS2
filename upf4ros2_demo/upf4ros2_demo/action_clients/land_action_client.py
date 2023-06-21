from upf4ros2_demo.action_clients.customaction_client import CustomActionClient


class LandActionClient(CustomActionClient):

    def __init__(self, node, feedback_callback, result_callback):
        super().__init__(node, feedback_callback, result_callback,"/landing")
        self.action_name="Land"


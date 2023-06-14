import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

from upf4ros2.ros2_interface_reader import ROS2InterfaceReader
from upf4ros2.ros2_interface_writer import ROS2InterfaceWriter
from upf_msgs import msg as msgs
from upf4ros2_demo.action_clients.take_off_action_client import TakeOffActionClient
from upf4ros2_demo.action_clients.land_action_client import LandActionClient
from upf4ros2_demo.action_clients.fly_action_client import FlyActionClient


from upf_msgs.action import (
    PDDLPlanOneShot
)

from upf_msgs.srv import (
    AddGoal,
    GetProblem,
    SetInitialValue
)

from upf_msgs.srv import PDDLPlanOneShot as PDDLPlanOneShotSrv

class PlanExecutorNode(Node):


    def __init__(self):
        super().__init__('plan_executor')

        self._problem_name = 'uav_problem'
        self._plan_result = {}
        self._actions = {}
        self._objects = {}
        self._plan = []

        self.declare_parameter('domain', '/pddl/uav_domain.pddl')
        self.declare_parameter('problem', '/pddl/generated_uav_instance.pddl')

        self._domain = self.get_parameter('domain')
        self._problem = self.get_parameter('problem')

        self._ros2_interface_writer = ROS2InterfaceWriter()
        self._ros2_interface_reader = ROS2InterfaceReader()

        # TODO: change first callback to feedback callback
        self._take_off_client = TakeOffActionClient(self,self.action_feedback_callback,self.finished_action_callback)
        self._land_client = LandActionClient(self,self.action_feedback_callback,self.finished_action_callback)
        self._fly_client = FlyActionClient(self,self.action_feedback_callback,self.finished_action_callback)

        self._plan_pddl_one_shot_client = ActionClient(
            self, 
            PDDLPlanOneShot, 
            'upf4ros2/action/planOneShotPDDL')
        
        self._get_problem = self.create_client(
            GetProblem, 'upf4ros2/srv/get_problem')
        self._add_goal = self.create_client(
            AddGoal, 'upf4ros2/srv/add_goal')
        self._set_initial_value = self.create_client(
            SetInitialValue, 'upf4ros2/srv/set_initial_value')
        self._plan_pddl_one_shot_client_srv = self.create_client(
            PDDLPlanOneShotSrv, 'upf4ros2/srv/planOneShotPDDL')
        
    
    def set_initial_value(self, fluent, object, value_fluent):
        """_summary_

        Args:
            fluent: _description_
            object: _description_
            value_fluent: _description_
        """        
        srv = SetInitialValue.Request()
        srv.problem_name = self._problem_name
        srv.expression = self._ros2_interface_writer.convert(fluent(*object))

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
        self.future = self._set_initial_value.call_async(srv)
        rclpy.spin_until_future_complete(self, self.future)

    def update_initial_state(self, action, parameters):
        """Updates the problem with all the effects from an action

        Args:
            action (upf_msgs/Action): the executed action with it's associated effects
            parameters: _description_
        """        
        self.get_logger().info("Updating initial state") 
        # maps identifiers of action parameters to their concrete instance
        # e.g. x : myuav, y : urbanArea, z : home
        paramMap = {action.parameters[i].name : parameters[i]  for i in range(len(parameters))}

        # loop through all effects of the action and update the initial state of the problem accordingly
        for effect in action.effects:
            fluent = effect.fluent.fluent()
            # there seems to be a bug in UPF where fluent.fluent() unpacks the arguments from the predicate declaration instead of  -> workaround: access the arguments directly with effect.fluent.args
            fluent_signature = [self._objects[paramMap[x.parameter().name]] for x in effect.fluent.args]
            value = effect.value.constant_value()
            self.set_initial_value(fluent, fluent_signature, value)
        self.get_logger().info(str(self.get_problem()))
    
    def remove_goal(self, goal):
        None
    
    def add_goal(self, goal):
        self.get_logger().info("Updating goals")
        srv = AddGoal.Request()
        srv.problem_name = self._problem_name
        upf_goal = msgs.Goal()
        upf_goal.goal = self._ros2_interface_writer.convert(goal)
        srv.goal.append(upf_goal)

        self._add_goal.wait_for_service()
        self.future = self._add_goal.call(srv)    

    def get_problem(self):
        """Retrieves the current state of the problem from the upf4ros problem manager

        Returns:
            upf_msgs/Problem: the current state of the problem
        """        
        srv = GetProblem.Request()
        srv.problem_name = self._problem_name

        self._get_problem.wait_for_service()
        self.future = self._get_problem.call_async(srv)
        rclpy.spin_until_future_complete(self, self.future)
        problem = self._ros2_interface_reader.convert(self.future.result().problem)
        #problem = self.future.result().problem
        return problem
    
    def get_plan_srv(self):
        """_summary_
        """        
        self.get_logger().info('Planning...')
        srv = PDDLPlanOneShotSrv.Request()
        srv.plan_request.mode = msgs.PDDLPlanRequest.FILE
        srv.plan_request.domain = (get_package_share_directory('upf4ros2_demo')
                                        + str(self._domain.value))
        srv.plan_request.problem = (get_package_share_directory('upf4ros2_demo')
                                        + str(self._problem.value))
        srv.plan_request.problem_name = self._problem_name

        self._plan_pddl_one_shot_client_srv.wait_for_service()
        self.future = self._plan_pddl_one_shot_client_srv.call_async(srv)
        rclpy.spin_until_future_complete(self, self.future)

        plan_result = self.future.result().plan_result
        # self.get_logger().info('Planned Steps are:' + str(plan_result.plan.actions))
        upfProblem = self.get_problem()
        
        self._objects = {upfProblem.all_objects[i].name : upfProblem.all_objects[i] for i in range(len(upfProblem.all_objects))}
        self._actions = {upfProblem.actions[i].name : upfProblem.actions[i] for i in range(len(upfProblem.actions))}
        self._plan = plan_result.plan.actions
    
    def finished_action_callback(self, action, params, result):
        self.get_logger().info("Completed action: " + action.action_name+"("+", ".join(params)+")")
        self.update_initial_state(self._actions[action.action_name], params)

        # TODO: add error handling
        # execute next action
        self.execute_plan()

    def action_feedback_callback(self, action, params, feedback):
        None

    def execute_plan(self):
        if len(self._plan) == 0:
            self.get_logger().info('Plan completed!')
            return
        action = self._plan.pop(0)
        actionName = action.action_name
        params = [x.symbol_atom[0] for x in action.parameters]
        self.get_logger().info("Next action: " + action.action_name+"("+", ".join(params)+")")

        if actionName == "take_off":
            self._take_off_client.send_action_goal(action, params)
        elif actionName == "land":
            self._land_client.send_action_goal(action, params)
        elif actionName == "fly":
            self._fly_client.send_action_goal(action, params)
        else:
            self.get_logger().info("Error! Received invalid action name")
            return

def main(args=None):
    rclpy.init(args=args)
    plan_executor_node = PlanExecutorNode()

    plan_executor_node.get_plan_srv()
    plan_executor_node.execute_plan()
    rclpy.spin(plan_executor_node)

    plan_executor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

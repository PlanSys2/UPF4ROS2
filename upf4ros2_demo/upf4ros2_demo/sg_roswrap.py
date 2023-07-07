import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from upf4ros2_demo.pddl_from_sg import ProblemMSG
from gtsg.monitorGame import MonitorSG
import numpy as np
import datetime
import pandas as pd
import os

from upf4ros2_demo_msgs.action import Mission
#from upf4ros2_demo_msgs.srv import CallMission

from std_msgs.msg import String

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleGlobalPosition

import time

from std_msgs.msg import Bool


class GameROSWrapper(Node):

    def __init__(self):
        super().__init__('game_manager')
        tmp_zeros = np.zeros((2, 4))
        mobis = MonitorSG(2, 4,
                      [[0, 0, 0, 0], [1, 1, 1, 1]],
                      [[[1 - 0.855, 0.855], [1 - 0.64, 0.64]],
                       [[1 - 0.64, 0.64], [1 - 0.35, 0.35]],
                       [[1 - 0.56, 0.56], [1 - 0.36, 0.36]],
                       [[1 - 0.42, 0.42], [1 - 0.42, 0.42]]],
                      tmp_zeros
                      )
        self.game=mobis
        self.get_logger().info("test1")
        self.problem=ProblemMSG(self.game,5)
        self.oldtime=0
        self.get_logger().info(f"{os.getcwd()}")
        self.df=pd.DataFrame([],columns=["drone","x","y","datetime"])
        self.df.to_csv("testdataPosi.csv")
        self.action_client_mission=ActionClient(self, Mission, 'mission')
       
    

    #Game solver: default
    def solvegame(self):
        self.get_logger().info("Start")
        self.problem.problem_gen((1,1,1,1),0)
        self.get_logger().info("End")
    
    #Gam solver with status argument
    def solvegame_status(self,status):
        self.get_logger().info("Start")
        self.problem.problem_gen(status,0)
        self.get_logger().info("End")
            
    #Action launcher with status as arg
    def launch_game(self,init_status):
        self.get_logger().info("Begin new round")
        goal_msg=Mission.Goal()
        goal_msg.init_status=init_status
        self.problem.problem_gen(tuple(init_status),0)
        self.get_logger().info("Game Generation end")
        self.get_logger().info("Waiting for action server")
        self.action_client_mission.wait_for_server()
        self.get_logger().info("Requesting action to plan executor")
        self._send_goal_future = self.action_client_mission.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        

    #future callback for action
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.mission_result_callback)
        
        
    
    def mission_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.final_status))
        self.launch_game(result.final_status)
    

def main(args=None):
    tmp_zeros = np.zeros((2, 4))
    mobis = MonitorSG(2, 4,
                      [[0, 0, 0, 0], [1, 1, 1, 1]],
                      [[[1 - 0.855, 0.855], [1 - 0.64, 0.64]],
                       [[1 - 0.64, 0.64], [1 - 0.35, 0.35]],
                       [[1 - 0.56, 0.56], [1 - 0.36, 0.36]],
                       [[1 - 0.42, 0.42], [1 - 0.42, 0.42]]],
                      tmp_zeros
                      )
    rclpy.init()
    gamenode=GameROSWrapper()
    gamenode.get_logger().info("Computing game policy")
    gamenode.solvegame()
    status=[0,0,0,1]
    gamenode.launch_game(status)
    rclpy.spin(gamenode)
    gamenode.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
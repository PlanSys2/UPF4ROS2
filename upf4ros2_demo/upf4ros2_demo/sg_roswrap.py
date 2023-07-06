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
        # self.local_pos_sub = self.create_subscription(
            # VehicleGlobalPosition,
            # '/fmu/vehicle_global_position/out',
            # self.listener_callbackbis,
            # 100)
        self.action_client_mission=ActionClient(self, Mission, 'mission')
       
    
    # def listener_callbackbis(self, msg):
        # if msg.timestamp-self.oldtime>5000000: #10 second: 10000000:
            # current_time=datetime.datetime.now()
            # tmpdf= pd.DataFrame({"drone":["d1"],"x":[msg.lon],'y':[msg.lat],"datetime":[current_time]})
            # tmpdf.to_csv('testdataPosi.csv', mode='a',header=False)
            # self.oldtime=msg.timestamp
            # self.get_logger().info(f"{msg.timestamp}")
            # self.get_logger().info(f"Longitude:{msg.lon},Latitude:{msg.lat}")

    

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
        
    #Callback for subscription
    def launch_newgame(self,msg):
        self.problem.problem_gen((0,1,0,0),0)
        self.get_logger().info('I heard: "%s"' % msg.data)
        msgbis=String()
        msgbis.data="GameFinishReload"
        self.newgame_pub.publish(msgbis)
        
        
    #Action launcher with status as arg
    def launch_game(self,init_status):
        self.get_logger().info("Begin Mission")
        goal_msg=Mission.Goal()
        goal_msg.init_status=init_status
    
        self.problem.problem_gen(tuple(init_status),0)
        self.get_logger().info("Gen end")
        self.action_client_mission.wait_for_server()
        self.get_logger().info("Wait End")
        self._send_goal_future = self.action_client_mission.send_goal_async(goal_msg)
        self.get_logger().info(f"send goal future")
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.get_logger().info(f"add_done_callback")
        
        

    #future callback for action
    def goal_response_callback(self, future):
        self.get_logger().info(f"begin goal_response_callback")
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self.get_logger().info(f"add get_result_callback")
        self._get_result_future.add_done_callback(self.get_result_callback)
        self.get_logger().info(f"end goal_response_callback")
        
        
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.final_status))
        self.launch_game(result.final_status)
        #self.current_status=result.final_status
        #return result.final_status
        #rclpy.shutdown()
    
    #Service launcher
    def send_request(self,status):
        self.req_cus.init_status=list(status)
        self.solvegame_status(status)
        self.get_logger().info("SolveGameEnd")
        self.future = self.client_mission.call_async(self.req_cus)



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
    gamenode.solvegame()
    gamenode.get_logger().info("Test1")
    status=[0,0,0,1]
    gamenode.launch_game(status)
    gamenode.get_logger().info(f"Test order")
    rclpy.spin(gamenode)
    gamenode.get_logger().info("Test2")

    gamenode.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
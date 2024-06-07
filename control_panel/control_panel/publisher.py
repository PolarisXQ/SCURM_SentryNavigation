import rclpy
from rclpy.node import Node
from rm_interfaces.msg import GameState
from auto_aim_interfaces.msg import Target
from std_msgs.msg import String,Bool,Int16,Int8
from geometry_msgs.msg import PoseStamped
import time


class Panel_Publisher(Node):

    def __init__(self,node_name='control_panel_pub'):
        super().__init__(node_name)
        self.game_state_publisher_=self.create_publisher(GameState,'game_state_sim',10)
        self.timer_=self.create_timer(1,self.timer_callback)

        
        self.game_state=GameState()
        self.game_state.game_progress=0
        self.game_state.stage_remain_time=32767
        self.game_state.current_hp=700
        self.game_state.projectile_allowance_17mm=400
        self.game_state.my_outpost_hp=1000
        self.game_state.enemy_outpost_hp=1000
        self.game_state.my_base_hp=1000
        self.game_state.enemy_base_hp=1000
        
    def set_game_state(self,game_progress,stage_remain_time,current_hp,projectile_allowance_17mm,my_outpost_hp,enemy_outpost_hp,my_base_hp,enemy_base_hp):
        self.game_state.game_progress=game_progress
        self.game_state.stage_remain_time=stage_remain_time
        self.game_state.current_hp=current_hp
        self.game_state.projectile_allowance_17mm=projectile_allowance_17mm
        self.game_state.my_outpost_hp=my_outpost_hp
        self.game_state.enemy_outpost_hp=enemy_outpost_hp
        self.game_state.my_base_hp=my_base_hp
        self.game_state.enemy_base_hp=enemy_base_hp

        
    def timer_callback(self):
        if(self.game_state.stage_remain_time>0):
            self.game_state.stage_remain_time=self.game_state.stage_remain_time-1
        else:
            self.game_state.game_progress=5
        
        

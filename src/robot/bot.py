#TODO: Fix paths
import sys
sys.path.append('C:/Users/Student/Documents/RLBot_IS/trajectory-optimization')

import math
import traceback

"""RLBot Imports"""
from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.game_state_util import (
    GameState,
    BallState,
    CarState,
    Physics,
    Vector3,
    Rotator,
    GameInfoState,
)

#TODO: Fix paths
"""Comms Imports"""
from src.communications.comms_protocol import CommsProtocol
from src.communications.server import Server
from src.communications.client import Client

"""Dynamics Imports"""
from state import State

class Bot(BaseAgent):
    def __init__(self, name, team, index):
        super().__init__(name, team, index)
        pass

    def initialize_agent(self):
        self.modify_game_state()
        

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        if(self.flag == True):
            self.set_game_state(self.game_state)
        self.flag = False

        return SimpleControllerState()

    def modify_game_state(self):
        s = Server(CommsProtocol.SERVER, CommsProtocol.PORT)
         #TODO: Run GUI here. On button push, client should send the message (do that within GUI functions)
        c = Client()
        try:
           
            #### REMOVE ####
            q = State()
            c.send_message(CommsProtocol.types["initialize_state"], np.array2string(q()))
            #### REMOVE ####
            
            msg = s.msg_queue.get()
            # msg is not passing a class
        
        except:
            traceback.print_exc()

        if msg.type == "initialize state":
            self.q = np.fromstring(msg.data)
            # TODO: Convert to Euler 
            car_state = CarState(
                boost_amount=100,
                physics=Physics(
                    location=Vector3(self.q.x, self.q.y, self.q.z),
                    velocity=Vector3(self.q.x_dot, self.q.y_dot, self.q.z_dot),
                    rotation=Rotator(0, 0, 0), # TODO: Take state values
                    angular_velocity=Vector3(self.q.phi_dot, self.q.theta_dot, self.q.psi_dot),
                ),
            )
            ball_state = BallState(Physics(location=Vector3(0, 0, None)))
            game_info_state = GameInfoState(game_speed=1)
            self.game_state = GameState(
                ball=ball_state, cars={self.index: car_state}, game_info=game_info_state
            )
            
            self.flag = True
        


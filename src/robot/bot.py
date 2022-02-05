#TODO: Fix paths
import sys
sys.path.append('C:/Users/Student/Documents/RLBot_IS/trajectory-optimization')

import math
import traceback
import numpy as np

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
from src.communications.comms_protocol import CommsProtocol
from src.communications.server import Server
from src.communications.client import Client

from state import State

class Bot(BaseAgent):
    def __init__(self, name, team, index):
        super().__init__(name, team, index)
        self.initialized = False
        self.q = State()

    def initialize_agent(self):
        self.s = Server(CommsProtocol.SERVER, CommsProtocol.PORT)
        
    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        if not self.s.msg_queue.empty():
            self.modify_game_state()
        
        return SimpleControllerState()

    def modify_game_state(self):
        print("Modifying game state")
        try:
            msg = self.s.msg_queue.get()
        
        except:
            traceback.print_exc()

        if msg.type == "initialize state":
            self.q.update_with_array(np.fromstring(msg.data.strip('[]'), count = 13, sep = ' '))
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
            
            self.set_game_state(self.game_state)

            self.initialized = True
            

        


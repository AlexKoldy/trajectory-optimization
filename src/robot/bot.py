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
        print("TEST0!")
        s = Server(CommsProtocol.SERVER, CommsProtocol.PORT)
        print("TEST1!")
    
        waiting = True
        while waiting:
            try:
                #TODO: Run GUI here. On button push, client should send the message (do that within GUI functions)
                q = State()
                c = Client()
                c.send_message(CommsProtocol.types["initialize"], q())
                
                msg = s.msg_queue.get()
                # msg is not passing a class
                self.q = msg.data
                print(f"Initialized state: {self.q()}")
                waiting = False

            except:
                traceback.print_exc()
                break
        
        # TODO: fill this with state information from the GUI
        car_state = CarState(
            boost_amount=87,
            physics=Physics(
                location=Vector3(x=100, y=100, z=100),
                rotation=Rotator(0, 0, 0),
                angular_velocity=Vector3(0, 0, 0),
            ),
        )
        ball_state = BallState(Physics(location=Vector3(0, 0, None)))
        game_info_state = GameInfoState(game_speed=1)
        game_state = GameState(
            ball=ball_state, cars={self.index: car_state}, game_info=game_info_state
        )
        self.set_game_state(game_state)

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        pass

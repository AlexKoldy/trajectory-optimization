# TODO: Fix paths
import sys

sys.path.append("C:/Users/Student/Documents/RLBot_IS/trajectory-optimization")

import math
import traceback
import numpy as np
import copy

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

from src.communications.comms_protocol import CommsProtocol
from src.communications.server import Server
from src.communications.client import Client
from state import State
from model import Model
from src.utilities.utils import LinAlgUtils as lau
from src.utilities.history import History


class Bot(BaseAgent):
    def __init__(self, name, team, index):
        super().__init__(name, team, index)
        self.initialized = False
        self.q = State()
        self.t = 0
        self.dt = 1 / 120
        self.model = Model(dt=self.dt, q=copy.deepcopy(self.q))
        self.history = History()

    def initialize_agent(self):
        self.s = Server(CommsProtocol.SERVER, CommsProtocol.PORT)

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        if not self.s.msg_queue.empty():
            self.modify_game_state()

        self.get_state()

        if t < 10:
            self.history.append_many_with_array(
                t=self.t, q=self.q(), q_m=self.model.q()
            )
        else:
            self.history.plot()

        controls = SimpleControllerState()
        controls.boost = True
        controls.ptich = -1

        u = np.array([0, -12.46, 0, 992])
        self.model.step(u=u)

        self.t += self.dt

        return SimpleControllerState()

    def get_state(self):
        """
        Retrieves state from game's rigid_body_tick and update state object
        """
        rbt = self.get_rigid_body_tick()
        self.q.x = rbt.players[self.index].state.location.x
        self.q.y = rbt.players[self.index].state.location.y
        self.q.z = rbt.players[self.index].state.location.z
        self.q.x_dot = rbt.players[self.index].state.velocity.x
        self.q.y_dot = rbt.players[self.index].state.velocity.y
        self.q.z_dot = rbt.players[self.index].state.velocity.z
        self.q.e0 = rbt.players[self.index].state.rotation.x
        self.q.e1 = rbt.players[self.index].state.rotation.y
        self.q.e2 = rbt.players[self.index].state.rotation.z
        self.q.e3 = rbt.players[self.index].state.rotation.w
        self.q.phi_dot = rbt.players[self.index].state.angular_velocity.x
        self.q.theta_dot = rbt.players[self.index].state.angular_velocity.y
        self.q.psi_dot = rbt.players[self.index].state.angular_velocity.z

    def modify_game_state(self):
        """
        Modifies game state
        """
        print("Modifying game state")
        try:
            msg = self.s.msg_queue.get()

        except:
            traceback.print_exc()

        if msg.type == "initialize state":
            self.q.update_with_array(
                np.fromstring(msg.data.strip("[]"), count=13, sep=" ")
            )
            self.model.q = self.q
            phi, theta, psi = lau.quaternion_to_euler(
                e0=self.q.e0, e1=self.q.e1, e2=self.q.e2, e3=self.q.e3
            )
            car_state = CarState(
                boost_amount=100,
                physics=Physics(
                    location=Vector3(self.q.x, self.q.y, self.q.z),
                    velocity=Vector3(self.q.x_dot, self.q.y_dot, self.q.z_dot),
                    rotation=Rotator(phi, theta, psi),
                    angular_velocity=Vector3(
                        self.q.phi_dot, self.q.theta_dot, self.q.psi_dot
                    ),
                ),
            )
            ball_state = BallState(Physics(location=Vector3(0, 0, None)))
            game_info_state = GameInfoState(game_speed=1)
            self.game_state = GameState(
                ball=ball_state, cars={self.index: car_state}, game_info=game_info_state
            )

            self.set_game_state(self.game_state)

            self.initialized = True

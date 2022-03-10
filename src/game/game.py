# TODO: Fix paths
import sys

sys.path.append("C:/Users/Student/Documents/RLBot_IS/trajectory-optimization")

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

from ball import Ball
from src.robot.bot import Bot
from src.communications.comms_protocol import CommsProtocol
from src.communications.server import Server
from src.communications.client import Client
from src.utilities.history import History
from src.utilities.lin_alg_utils import LinAlgUtils as lau


class Game(BaseAgent):
    def __init__(self, name, team, index):
        super().__init__(name, team, index)
        self.bot = Bot()  # handles robot
        self.ball = Ball()  # handles ball
        self.t = 0  # game time [s]
        self.dt = 1 / 120  # time step [s]
        self.history = History()  # game state history
        self.sim_state = "WAITING_TO_INITIALIZE"  # game's state machine
        self.saved = False
        # self.initialized = False

    def initialize_agent(self):
        self.server = Server(CommsProtocol.SERVER, CommsProtocol.PORT)

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        if not self.server.msg_queue.empty():
            try:
                msg = self.server.msg_queue.get()
            except:
                traceback.print_exc()
            if msg.type == "initialize state":
                self.sim_state = "INITIALIZING_STATE"
            elif msg.type == "modify state":
                self.sim_state = "MODIFYING_STATE"
        if self.sim_state == "READY_TO_INITIALIZE":
            self.t = 0
        elif self.sim_state == "INITIALIZING_STATE":
            self.modify_game_state(
                q_bot=np.fromstring(msg.data.strip("[]"), count=13, sep=" ")
            )
            self.sim_state = "RUNNING_GAME"
        elif self.sim_state == "RUNNING_GAME":
            q_bot, q_ball = self.get_state()  # get actual bot and ball state
            self.bot.q.update_with_array(state_array=q_bot)  # update bot's state
            if self.t < 20:
                self.history.append_many_with_array(
                    t=self.t, q=self.bot.q(), q_m=self.bot.model.q()
                )  # append gamestate to history
            else:
                self.saved = self.history.save()
                print("Ready to plot!")
                self.sim_state = "READY_TO_INITIALIZE"
            quat_des = lau.euler_to_quaternion(phi=0, theta=0, psi=0)
            controls = self.bot.step(quat_des=quat_des)
            self.t += self.dt
            return controls
        elif self.sim_state == "MODIFYING_STATE":
            self.modify_game_state(
                np.fromstring(q_bot=msg.data.strip("[]"), count=13, sep=" ")
            )

        return SimpleControllerState()

    def get_state(self):
        """
        Retrieves state from game's rigid_body_tick and update state object

        Returns:
            q_bot (np.array): bot's actual state within the game
            q_ball (np.array): ball's actual state within the game
        """
        rbt = self.get_rigid_body_tick()
        x = rbt.players[self.index].state.location.x
        y = rbt.players[self.index].state.location.y
        z = rbt.players[self.index].state.location.z
        x_dot = rbt.players[self.index].state.velocity.x
        y_dot = rbt.players[self.index].state.velocity.y
        z_dot = rbt.players[self.index].state.velocity.z
        e0 = rbt.players[self.index].state.rotation.x
        e1 = rbt.players[self.index].state.rotation.y
        e2 = rbt.players[self.index].state.rotation.z
        e3 = rbt.players[self.index].state.rotation.w
        phi_dot = rbt.players[self.index].state.angular_velocity.x
        theta_dot = rbt.players[self.index].state.angular_velocity.y
        psi_dot = rbt.players[self.index].state.angular_velocity.z

        q_bot = np.array(
            [x, y, z, x_dot, y_dot, z_dot, e0, e1, e2, e3, phi_dot, theta_dot, psi_dot]
        )
        q_ball = None  # TODO: Fix
        return q_bot, q_ball

    def modify_game_state(self, q_bot: np.array):
        """
        Modifies game state

        Args:
            q_bot (np.array): new bot state to set
        """
        car_state = self.bot.set_state(q=q_bot)
        ball_state = self.ball.set_state()
        game_info_state = GameInfoState(world_gravity_z=0.00001, game_speed=1)
        self.game_state = GameState(
            ball=ball_state, cars={self.index: car_state}, game_info=game_info_state
        )
        self.set_game_state(self.game_state)

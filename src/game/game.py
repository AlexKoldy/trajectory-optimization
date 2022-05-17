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
from src.utilities.utils import convert_data


class Game(BaseAgent):
    def __init__(self, name, team, index):
        super().__init__(name, team, index)
        self.bot = Bot()  # handles robot
        self.ball = Ball()  # handles ball
        self.t = 0  # game time [s]
        self.dt = 1 / 120  # time step [s]
        self.history = History()  # game state history
        self.states = {
            0: "WAITING_TO_INITIALIZE",
            1: "MODIFYING_STATE",
            2: "RUNNING_GAME",
        }  # game's state machine
        self.sim_state = 0
        self.saved = False

    def initialize_agent(self):
        self.sim_state = 0
        self.server = Server(CommsProtocol.SERVER, CommsProtocol.PORT)

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        if self.server.msg_queue.empty() == False:
            try:
                self.msg = self.server.msg_queue.get()
                if self.msg.type == "modify state":
                    self.sim_state = 1
            except:
                traceback.print_exc()
        if self.sim_state == 0:
            self.saved = False
            self.t = 0
        elif self.sim_state == 1:
            self.t = 0
            self.bot.count = 0
            data = self.msg.data
            q_bot, quat_des, controller_coefficients, g = convert_data(data=data)
            self.modify_game_state(
                q_bot=q_bot,
                quat_des=quat_des,
                g=g,
                controller_coefficients=controller_coefficients,
            )
            self.sim_state = 2
        elif self.sim_state == 2:
            q_bot, q_ball = self.get_state()  # get actual bot and ball state
            theta = packet.game_cars[0].physics.rotation.pitch
            self.bot.q.update_with_array(state_array=q_bot)
            if self.t < 10:
                self.history.append_many_with_array(
                    t=self.t, q=self.bot.q(), q_m=self.bot.model.q()
                )  # append gamestate to history
            else:
                # self.saved = self.history.save()
                print("Ready to plot!")
                self.sim_state = 0
            controls = self.bot.step(quat_des=self.quat_des, theta=theta)
            self.t += self.dt
            return controls
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

    def modify_game_state(
        self,
        q_bot: np.array,
        quat_des: np.array,
        g: float,
        controller_coefficients: np.array,
    ):
        """
        Modifies game state

        Args:
            q_bot (np.array): new bot state to set
        """
        # TODO: MAKE THIS FUNCTIONS ARGS CLEANER; EDIT DOCUMENTATION
        self.bot.p2_cascade.set_constants(
            P_quat=controller_coefficients[0], P_omega=controller_coefficients[1]
        )
        self.quat_des = quat_des
        car_state = self.bot.set_state(q=q_bot)
        ball_state = self.ball.set_state()
        g = -660
        # g = 0.0001
        game_info_state = GameInfoState(world_gravity_z=g, game_speed=1)
        self.game_state = GameState(
            ball=ball_state, cars={self.index: car_state}, game_info=game_info_state
        )
        self.set_game_state(self.game_state)

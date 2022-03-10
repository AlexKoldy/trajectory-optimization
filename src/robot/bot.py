# TODO: Fix paths
import sys

sys.path.append("C:/Users/Student/Documents/RLBot_IS/trajectory-optimization")

import numpy as np
import copy

from rlbot.agents.base_agent import SimpleControllerState
from rlbot.utils.game_state_util import (
    CarState,
    Physics,
    Vector3,
    Rotator,
)

from src.robot.state import State
from src.robot.model import Model
from src.controllers.p2 import P2
from src.utilities.lin_alg_utils import LinAlgUtils as lau


class Bot:
    def __init__(self):
        self.q = State()
        self.dt = 1 / 120
        self.model = Model(dt=self.dt, q=copy.deepcopy(self.q))
        self.controller = P2(dt=self.dt, P_quat=10, P_omega=0.01)

    def set_state(self, q: np.array) -> CarState:
        """
        Sets the bot's state

        Args:
            q (np.array): new bot state
        """
        self.q.update_with_array(state_array=q)
        self.model.q = copy.deepcopy(self.q)
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
        return car_state

    def step(self, quat_des: np.array) -> SimpleControllerState:
        controls = SimpleControllerState()
        quat = self.q()[6:10]
        omega = self.q()[10:]
        tau = self.controller.step(quat=quat, quat_des=quat_des, omega=omega)
        tau = tau / np.linalg.norm(tau)
        controls.roll = tau[0]
        controls.pitch = tau[1]
        controls.yaw = tau[2]
        u = np.array([tau[0], tau[1], tau[2], 0])
        self.model.step(u=u)
        return controls

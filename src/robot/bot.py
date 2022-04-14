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
from src.controllers.geometric_controller import GeometricController
from src.controllers.bang_bang import BangBang
from src.utilities.lin_alg_utils import LinAlgUtils as lau


class Bot:
    def __init__(self):
        self.q = State()
        self.dt = 1 / 120
        self.model = Model(dt=self.dt, q=copy.deepcopy(self.q))
        self.p2 = P2(dt=self.dt, P_quat=0, P_omega=0)
        self.geometric_controller = GeometricController()
        self.bang_bang = BangBang()

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
                rotation=Rotator(theta, psi, phi),
                angular_velocity=Vector3(
                    self.q.phi_dot, self.q.theta_dot, self.q.psi_dot
                ),
            ),
        )
        return car_state

    def step(self, quat_des: np.array) -> SimpleControllerState:
        controls = SimpleControllerState()
        v = self.q()[3:6]
        quat = self.q()[6:10]
        omega = self.q()[10:]
        # print(f"v: {v}")
        # TODO: Remove
        v_des = np.array([50, 50, 0])
        v_error = v_des - v
        # print(f"v_error: {v_error}")
        # v_des = np.hstack((v_des, 0))
        # v_des = lau.quat_body_to_world(quat_0=quat, quat_1=v_des)[:3]
        # quat_des = self.geometric_controller.step(v=v, v_des=v_des, quat=quat)
        # print(f"quat_des: {quat_des}")
        boost = self.bang_bang.step(quat=quat, v=v, v_des=v_des)
        tau = self.p2.step(quat=quat, quat_des=quat_des, omega=omega)
        controls.roll = tau[0] / 36.07956616966136
        controls.pitch = tau[1] / 12.14599781908070  # TODO: Check sign
        controls.yaw = tau[2] / 8.91962804287785
        controls.roll = np.clip(controls.roll, -1, 1)
        controls.pitch = np.clip(controls.pitch, -1, 1)
        controls.yaw = np.clip(controls.yaw, -1, 1)
        boost = 0
        controls.boost = boost
        u = np.array([controls.roll, controls.pitch, controls.yaw, controls.boost])
        self.model.step(u=u)
        return controls

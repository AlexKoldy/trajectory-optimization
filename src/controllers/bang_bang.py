import sys

sys.path.append("C:/Users/Student/Documents/RLBot_IS/trajectory-optimization")

import numpy as np

from src.utilities.lin_alg_utils import LinAlgUtils as lau


class BangBang:
    def __init__(self):
        self.dt = 1 / 120  # timestep [s]
        self.a_boost = np.array(
            [991.666, 0, 0]
        )  # boost acceleration in body frame [uu/s^2]
        self.g = np.array([0, 0, -650])  # gravity [uu/s^2]
        self.drag = 0.03

    def step(self, quat: np.array, v: np.array, v_des: np.array) -> float:
        """
        TODO
        Determines input boost to the car_state

        Args:
            speed (float): current speed of the vehicle
            speed_des (float): desired speed of the vehicle

        Returns:
            boost (int): whether or not to use boost (0 or 1)
        """
        a_boost = np.hstack((self.a_boost, 0))
        a_boost = lau.quat_body_to_world(
            quat_0=quat, quat_1=a_boost
        )  # boost acceleration in world frame [uu/s^2]
        a_total = self.g
        v_on = v + (a_total + a_boost[:3]) * self.dt
        v_off = v + a_total * self.dt
        v_error_on = v_des - v_on
        v_error_off = v_des - v_off
        if np.linalg.norm(v_error_on) < np.linalg.norm(v_error_off):
            boost = 1
        else:
            boost = 0
        return boost


if __name__ == "__main__":
    bangbang = BangBang()
    quat = np.array([0, 0, 0, 1])
    v = np.array([90, 0, 0])
    v_des = np.array([100, 0, -20])
    print(bangbang.step(quat, v, v_des))

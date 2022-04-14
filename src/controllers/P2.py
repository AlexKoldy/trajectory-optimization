import sys

sys.path.append("C:/Users/Student/Documents/RLBot_IS/trajectory-optimization")

import numpy as np

from src.utilities.lin_alg_utils import LinAlgUtils as lau
from pyquaternion import Quaternion


class P2:
    def __init__(self, dt: float, P_quat: float = 0, P_omega: float = 0):
        self.dt = dt  # timestep [s]
        self.P_quat = P_quat
        self.P_omega = P_omega

    def set_constants(self, P_quat: float = None, P_omega: float = None):
        """
        Sets P2 constants to new values. For tuning purposes only

        Args:
            P_quat (float): new outer loop constant (optional)
            P_omega (float): new inner loop constant loop (optional)
        """
        if P_quat:
            self.P_quat = P_quat
        if P_omega:
            self.P_omega = P_omega

    def step(self, quat_des: np.array, quat: np.array, omega: np.array) -> np.array:
        """
        Determines input to plant using full-state feedback

        Args:
            quat_des (np.array): desired quaternion
            quat (np.array): measured quaternion
            omega (np.array): angular velocity
        """
        quat_des = lau.quat_normalize(quat_des)
        quat_des[0] *= -1
        quat_des[1] *= -1
        quat = lau.quat_normalize(quat)
        print(f"quat_des: {quat_des}")
        print(f"quat: {quat}")
        quat_conj = lau.quat_conjugate(quat=quat)  # conjugate of measured quaternion
        print(f"quat_conj: {quat_conj}")
        quat_error = lau.quat_multiply(quat_0=quat_des, quat_1=quat_conj)
        if quat_error[3] < 0:
            quat_error *= -1
        u = -self.P_quat * quat_error[:3] - self.P_omega * omega
        return u

import sys

sys.path.append("C:/Users/Student/Documents/RLBot_IS/trajectory-optimization")

import numpy as np

from src.utilities.lin_alg_utils import LinAlgUtils as lau


class P2Cascade:
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
        if not P_quat == None:
            self.P_quat = P_quat
        if not P_omega == None:
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
        quat = lau.quat_normalize(quat)
        quat_conj = lau.quat_conjugate(quat=quat)  # conjugate of measured quaternion
        quat_error = lau.quat_multiply(quat_0=quat_des, quat_1=quat_conj)
        print(f"Quaternion Error: {quat_error}")
        if quat_error[3] < 0:
            quat_error *= -1
        omega_des = -self.P_quat * quat_error[:3]
        omega_error = omega_des - omega
        u = -self.P_omega * omega_error
        return u

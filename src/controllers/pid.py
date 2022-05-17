import sys

sys.path.append("C:/Users/Student/Documents/RLBot_IS/trajectory-optimization")

import numpy as np

from src.utilities.lin_alg_utils import LinAlgUtils as lau
from pyquaternion import Quaternion


class PID:
    def __init__(self, dt: float, P: float = 0, I: float = 0, D: float = 0):
        self.dt = dt  # timestep [s]
        self.P = P
        self.I = I
        self.D = D
        self.e_sum = 0
        self.previous = 0
        self.e_prev = 0

    def set_constants(self, P: float = None, I: float = None, D: float = None):
        """
        Sets P2 constants to new values. For tuning purposes only

        Args:
            P (float): new proportional constant (optional)
            I (float): new integral constant (optional)
            D (float): new derivative constant (optional)
        """
        if not P == None:
            self.P = P
        if not I == None:
            self.I = I
        if not D == None:
            self.D = D

    def step(self, desired: float, actual: float) -> float():
        """
        Determines input to plant using pid equation

        Args:
            desired (float): desired value
            actual (float): real/measured value
        """
        e = desired - actual
        # print(e)
        self.e_sum += e * self.dt
        e_dot = (e - self.e_prev) / self.dt
        # print(f"test2: {type(e)}")
        u = self.P * e + self.I * self.e_sum + self.D * e_dot

        self.e_prev = e
        self.previous = actual
        return u

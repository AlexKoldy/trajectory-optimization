import sys

sys.path.append("C:/Users/Student/Documents/RLBot_IS/trajectory-optimization")

import numpy as np

from src.controllers.pid import PID


class Cascaded_PID:
    def __init__(
        self,
        dt: float,
        k_p_inner: float = 0,  # inner loop proportional constant
        k_i_inner: float = 0,  # inner loop integral constant
        k_d_inner: float = 0,  # inner loop derivative constant
        k_p_outer: float = 0,  # outer loop proportional constant
        k_i_outer: float = 0,  # outer loop integral constant
        k_d_outer: float = 0,  # outer loop derivative constant
    ):
        self.inner_pid = PID(dt=dt, k_p=k_p_inner, k_i=k_i_inner, k_d=k_d_inner)
        self.outer_pid = PID(dt=dt, k_p=k_p_outer, k_i=k_i_outer, k_d=k_d_inner)
        self.dt = dt

    def set_constants(
        self,
        k_p_inner: float = None,
        k_i_inner: float = None,
        k_d_inner: float = None,
        k_p_outer: float = None,
        k_i_outer: float = None,
        k_d_outer: float = None,
    ):
        """
        Sets PID constants to new values. For tuning purposes only

        Args:
            k_p_inner (float): new inner proportional constant (optional)
            k_i_inner (float): new inner integral constant (optional)
            k_d_inner (float): new inner derivative constant (optional)
            k_p_outer (float): new outer proportional constant (optional)
            k_i_outer (float): new outer integral constant (optional)
            k_d_outer (float): new outer derivative constant (optional)
        """
        self.inner_pid.set_constants(k_p=k_p_inner, k_i=k_i_inner, k_d=k_d_inner)
        self.outer_pid.set_constants(k_p=k_p_outer, k_i=k_i_outer, k_d=k_d_outer)

    def step(self, x_des: np.array, x: np.array, x_dot: np.array) -> np.array:
        """
        Determines input to plant

        Args:
            x_des (np.array): desired behavior
            x (np.array): current behavior
            x_dot (np.array): current behavior's velocity

        Returns:
            u (np.array): input
        """
        x_dot_des = self.outer_pid(
            x_des=x_des, x=x
        )  # get desired velocity of behavior to feed to inner loop
        u = self.inner_pid(x_des=x_dot_des, x=x_dot)
        return u

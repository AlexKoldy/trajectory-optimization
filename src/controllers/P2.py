import sys

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

import numpy as np


class PID:
    def __init__(self, dt: double, k_p: float = 0, k_i: float = 0, k_d: float = 0):
        self.k_p = k_p  # proportional constant; units vary
        self.k_i = k_i  # integral constant; units vary
        self.k_d = k_d  # derivative constant; units vary
        self.dt = dt  # timestep [s]
        self.e_sum = 0  # error sum

    def set_constants(self, k_p: float = None, k_i: float = None, k_d: float = None):
        """
        Sets PID constants to new values. For tuning purposes only

        Args:
            k_p (float): new proportional constant (optional)
            k_i (float): new integral constant (optional)
            k_d (float): new derivative constant (optional)
        """
        if k_p:
            self.k_p = k_p
        if k_i:
            self.k_i = k_i
        if k_d:
            self.k_d = k_d

    def step(self, x_des: np.array, x: np.array) -> np.array:
        """
        Determines input to plant/second controller

        Args:
            x_des (np.array): desired behavior
            x (np.array): current behavior

        Returns:
            u (np.array): input
        """
        if not self.x_prev:  # for first step only
            self.x_prev = x
        e = x_des - x  # error
        if not self.e_prev:  # for first step only
            self.e_prev = e
        self.e_sum += e * self.dt  # integral of error
        e_dot = (e - self.e_prev) / self.dt  # derivative of error
        u = self.k_p * e + self.k_i * self.e_sum * self.k_d * e_dot  # PID formula
        return u

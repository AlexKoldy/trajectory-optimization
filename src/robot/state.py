import numpy as np


class State:
    def __init__(
        self,
        x=0,  # x-position [uu]
        y=0,  # y-position [uu]
        z=0,  # z-position [uu]
        x_dot=0,  # x-velocity [uu/s]
        y_dot=0,  # y-velocity [uu/s]
        z_dot=0,  # z-velocity [uu/s]
        e0=0,  # quaternion 0
        e1=0,  # quaternion 1
        e2=0,  # quaternion 2
        e3=0,  # quaternion 3
        phi_dot=0,  # roll rate [rad/s]
        theta_dot=0,  # pitch rate [rad/s]
        psi_dot=0,  # yaw rate [rad/s]
    ):
        self.x = x
        self.y = y
        self.z = z
        self.x_dot = x_dot
        self.y_dot = y_dot
        self.z_dot = z_dot
        self.e0 = e0
        self.e1 = e1
        self.e2 = e2
        self.e3 = e3
        self.phi_dot = phi_dot
        self.theta_dot = theta_dot
        self.psi_dot = psi_dot

    """
	Return entire state as 1D numpy array
	"""

    def __call__(self):
        return np.array(
            [
                self.x,
                self.y,
                self.z,
                self.x_dot,
                self.y_dot,
                self.z_dot,
                self.e0,
                self.e1,
                self.e2,
                self.e3,
                self.phi_dot,
                self.theta_dot,
                self.psi_dot,
            ]
        )

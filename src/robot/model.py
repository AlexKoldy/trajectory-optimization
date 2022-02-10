import sys

sys.path.append("C:/Users/Student/Documents/RLBot_IS/trajectory-optimization")

import numpy as np

from src.robot.state import State
from src.utilities.utils import quaternion_to_rotation


class Model:
    def __init__(self, q: State):
        self.q = q  # Predicted state of bot based off dynamic model
        self.T_phi = -36.07956616966136  # torque coefficient for roll
        self.T_theta = -12.14599781908070  # torque coefficient for pitch
        self.T_psi = 8.91962804287785  # torque coefficient for yaw
        self.D_phi = -4.47166302201591  # drag coefficient for roll
        self.D_theta = -2.798194258050845  # drag coefficient for pitch
        self.D_psi = -1.886491900437232  # drag coefficient for yaw
        self.m = 1  # TODO: units, real value

    def step(self, dt: float, u: np.array):
        """
        Predicts new state based off of input conditions and timestep

        Args:
            dt (float): timestep (s)
            u (np.array): user input array
        """
        # TODO: Ensure proper coordinate frame instantiation
        # TODO: Check velocity and position predictions
        # TODO: Implement boundary conditions (i.e. max velocity, collisions)
        p = self.q()[:3]  # position of bot in world frame [uu]
        v = self.q()[3:6]  # velocity of bot in body frame [uu/s]
        quat = self.q()[6:10]  # quaternion describing bot's orientation
        omega = self.q()[10:]  # angular velocity of bot in body frame [rad/s]

        # Convert quaternion into a rotation matrix
        Theta = quaternion_to_rotation(self.q.e0, self.q.e1, self.q.e2, self.q.e3)

        T = np.array(
            [[self.T_phi, 0, 0], [0, self.T_theta, 0], [0, 0, self.T_psi]], dtype=object
        )
        D = np.array(
            [
                [self.D_phi, 0, 0],
                [0, self.D_theta * (1 - np.abs(u[1])), 0],
                [0, 0, self.D_psi * (1 - np.abs(u[2]))],
            ],
            dtype=object,
        )

        # Calculate net torque on car
        tau = Theta @ (T @ u[:3] + D @ Theta.T @ omega)

        # Predict angular velocity at next timestep
        omega_new = omega + tau * dt
        omega_new = (1 / 2) * (omega + omega_new)
        omega = omega_new.flatten()

        # Predict new orientation
        Omega = np.array(
            [
                [0, omega[0], omega[1], omega[2]],
                [-omega[0], 0, omega[2], omega[1]],
                [-omega[1], -omega[2], 0, omega[0]],
                [-omega[2], omega[1], -omega[0], 0],
            ],
            dtype=object,
        )
        alpha = np.linalg.norm(omega) * dt
        quat = (
            quat
            + ((np.sin(alpha) / alpha) * Omega * dt) @ quat
            + (((1 - np.cos(alpha)) / alpha**2) * (Omega @ Omega) * dt**2) @ quat
        )

        # TODO: Get g from somewhere else
        # Convert gravity to body frame
        g = np.array([0, 0, -700])  # gravity in the world frame [uu/s^2]
        R = quaternion_to_rotation(e0=quat[0], e1=quat[1], e2=quat[2], e3=quat[3])
        g = R @ g  # gravity in the body frame [uu/s^2]

        # Equations of motion with respect to bot's center of mass
        x_ddot = u[3] / self.m + g[0]
        y_ddot = g[1]
        z_ddot = g[2]

        # Predict new velocity and convert to world frame
        a = np.array(
            [x_ddot, y_ddot, z_ddot]
        )  # acceleration of bot in body frame [uu/s^2]
        v = v + a * dt
        v = np.linalg.inv(R) @ v  # velocity of bot in world frame [uu/s]

        # Predict new position in world frame
        p = p + v * dt

        # Update bot's state
        self.q.update_with_array(
            np.array(
                [
                    p[0],
                    p[1],
                    p[2],
                    v[0],
                    v[1],
                    v[2],
                    quat[0],
                    quat[1],
                    quat[2],
                    quat[3],
                    omega[0],
                    omega[1],
                    omega[2],
                ],
                dtype=object,
            )
        )


# q = State()
# model = Model(q)
# print(model.q())
# model.step(dt=0.1, u=np.array([0, 0, 1, 1]))
# print(model.q())

import sys

sys.path.append("C:/Users/Student/Documents/RLBot_IS/trajectory-optimization")

import numpy as np

from src.robot.state import State
from src.utilities.utils import LinAlgUtils as lau
from src.utilities.integrators import get_integrator


class Model:
    def __init__(self, dt: float, q: State):
        self.q = q  # Predicted state of bot based off dynamic model
        self.T_phi = -36.07956616966136  # torque coefficient for roll
        self.T_theta = -12.14599781908070  # torque coefficient for pitch
        self.T_psi = 8.91962804287785  # torque coefficient for yaw
        self.D_phi = -4.47166302201591  # drag coefficient for roll
        self.D_theta = -2.798194258050845  # drag coefficient for pitch
        self.D_psi = -1.886491900437232  # drag coefficient for yaw
        self.m = 1  # TODO: units, real value

        self.T = np.array(
            [[self.T_phi, 0, 0], [0, self.T_theta, 0], [0, 0, self.T_psi]],
            dtype=np.float32,
        )

        self.g = np.array([0, 0, -660])  # gravity in world frame [uu/s^2]

        self.dt = dt  # timestep [s]
        self.intg = get_integrator(dt=self.dt, model=self.q_dot)

    def q_dot(self, q: np.array, u: np.array) -> np.array:
        """
        Calculate derivative of state for prediction

        Args:
            q (np.array): state
            u (np.array): torque and boost inputs

        Returns:
            q_dot (np.array): state derivative
        """
        p = q[:3]  # position of bot in world frame [uu]
        v = q[3:6]  # velocity of bot in world frame [uu/s]
        quat = q[6:10]  # quaternion describing bot's orientation
        omega = q[10:]  # angular velocity of bot in body frame [rad/s]

        quat = lau.quat_normalize(quat)
        D = np.array(
            [
                [self.D_phi, 0, 0],
                [0, self.D_theta * (1 - np.abs(u[1])), 0],
                [0, 0, self.D_psi * (1 - np.abs(u[2]))],
            ],
            dtype=np.float32,
        )
        omega_dot = self.T @ u[:3] + D @ omega  # angular acceleration [rad/s^2]
        omega = np.hstack((omega, 0))  # convert angular velocity to vector of size 4
        quat_dot = (1 / 2) * lau.quat_multiply(quat_0=omega, quat_1=quat)

        boost = np.array([u[3], 0, 0, 0])  # boost in pure quaternion form in body frame
        boost = lau.quat_multiply(
            quat_0=lau.quat_multiply(quat_0=quat, quat_1=boost),
            quat_1=lau.quat_conjugate(quat),
        )  # boost in pure quaternion form in world frame
        boost = boost[0:3]
        v_dot = boost + self.g  # acceleration of car in world frame [uu/s^2]

        return np.array(
            [
                v[0],
                v[1],
                v[2],
                v_dot[0],
                v_dot[1],
                v_dot[2],
                quat_dot[0],
                quat_dot[1],
                quat_dot[2],
                quat_dot[3],
                omega_dot[0],
                omega_dot[1],
                omega_dot[2],
            ],
            dtype=np.float32,
        )

    # Same direction of when I update magnitude cannot exceed 5.5
    # Add drag
    def step(self, u: np.array):
        """
        Predicts state at next timestep

        Args:
            u (np.array): torque and boost inputs
        """
        q_next = self.intg.step(x=self.q(), u=u)
        omega_next = q_next[10:]
        omega_mag = np.linalg.norm(omega_next)
        if omega_mag > 5.5:
            omega_hat = omega_next / omega_mag
            omega_hat *= 5.5
            q_next[10:] = omega_hat[:]
        self.q.update_with_array(q_next)


if __name__ == "__main__":
    q = State()
    model = Model(dt=1 / 120, q=q)
    t = 0
    u = np.array([0, 0.1, 0, 992])
    while t < 5:
        model.step(u)
        t += 1 / 120

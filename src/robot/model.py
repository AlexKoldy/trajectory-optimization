import sys

sys.path.append("C:/Users/Student/Documents/RLBot_IS/trajectory-optimization")

import numpy as np

from src.robot.state import State
from src.utilities.utils import (
    quaternion_to_rotation,
    quat_multiply,
    quat_conjugate,
    quat_normalize,
)
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

        """
        self.D = np.array(
            [
                [self.D_phi, 0, 0],
                [0, self.D_theta * (1 - np.abs(u[1])), 0],
                [0, 0, self.D_psi * (1 - np.abs(u[2]))],
            ],
            dtype=np.float32,
        )
        """

        self.g = np.array([0, 0, -660])  # gravity in world frame [uu/s^2]

        self.dt = dt  # timestep [s]
        self.intg = get_integrator(dt=self.dt, model=self.q_dot)

    def q_dot(self, q: np.array, u: np.array) -> np.array:
        p = q[:3]  # position of bot in world frame [uu]
        v = q[3:6]  # velocity of bot in world frame [uu/s]
        quat = q[6:10]  # quaternion describing bot's orientation
        omega = q[10:]  # angular velocity of bot in body frame [rad/s]

        quat = quat_normalize(quat)

        omega_dot = self.T @ u[:3]
        omega = np.hstack((omega, 0))
        quat_dot = (1 / 2) * quat_multiply(quat_0=omega, quat_1=quat)

        boost = np.array([u[3], 0, 0, 0])  # boost in pure quaternion form in body frame
        boost = quat_multiply(
            quat_0=quat_multiply(quat_0=quat, quat_1=boost), quat_1=quat_conjugate(quat)
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

    def step(self, u: np.array):
        self.q.update_with_array(self.intg.step(x=self.q(), u=u))

        # print(f"State: {self.q()}")

    def plot(self, t_history: list, x_history: list, y_history: list, z_history: list):
        import matplotlib.pyplot as plt

        plt.figure()
        plt.plot(t_history, x_history, label="x")
        plt.plot(t_history, y_history, label="y")
        plt.plot(t_history, z_history, label="z")
        plt.legend()
        plt.show()


q = State()
model = Model(dt=0.05, q=q)
u = np.array([0, 0, 2, 100])
t_history = []
t_history.append(0)
x_history = []
x_history.append(model.q.x)
y_history = []
y_history.append(model.q.y)
z_history = []
z_history.append(model.q.z)
t = 0

for num_timesteps in range(100):
    model.step(u)
    t += 0.05
    t_history.append(t)
    x_history.append(model.q.x)
    y_history.append(model.q.y)
    z_history.append(model.q.z)

# model.plot(
#   t_history=t_history, x_history=x_history, y_history=y_history, z_history=z_history
# )

import matplotlib.pyplot as plt

plt.figure()
plt.plot(t_history, x_history, label="x")
plt.plot(t_history, y_history, label="y")
plt.plot(t_history, z_history, label="z")
plt.ylim(0, 100)
plt.legend()
plt.show()

# print(model.q())
# model.step(dt=0.1, u=np.array([0, 0, 1, 1]))
# print(model.q())

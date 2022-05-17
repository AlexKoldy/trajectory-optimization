# TODO: Fix paths
import sys

sys.path.append("C:/Users/Student/Documents/RLBot_IS/trajectory-optimization")

from typing import List
import numpy as np
import copy

from rlbot.agents.base_agent import SimpleControllerState
from rlbot.utils.game_state_util import (
    CarState,
    Physics,
    Vector3,
    Rotator,
)
from pyquaternion import Quaternion

from src.robot.state import State
from src.robot.model import Model
from src.controllers.pid import PID
from src.controllers.p2_cascade import P2Cascade
from src.controllers.geometric_controller import GeometricController
from src.controllers.bang_bang import BangBang
from src.utilities.lin_alg_utils import LinAlgUtils as lau
from src.utilities.history import History


class Bot:
    def __init__(self):
        self.q = State()
        self.dt = 1 / 120
        self.model = Model(dt=self.dt, q=copy.deepcopy(self.q))
        self.t = 0  # [s]

        # Controllers
        self.pid_theta = PID(dt=self.dt, P=3, I=0, D=0)
        self.pid_omega = PID(dt=self.dt, P=20, I=0, D=0)
        self.p2_cascade = P2Cascade(dt=self.dt, P_quat=0, P_omega=0)
        self.geometric_controller = GeometricController()
        self.bang_bang = BangBang()

        # Trajectory
        self.history = History()
        (
            t_traj,
            y_traj,
            z_traj,
            y_dot_traj,
            z_dot_traj,
            theta_traj,
            theta_dot_traj,
            boost_traj,
            pitch_traj,
        ) = self.load_trajectory()
        (
            self.num_timesteps,
            self.t_traj,
            self.y_traj,
            self.z_traj,
            self.y_dot_traj,
            self.z_dot_traj,
            self.theta_traj,
            self.theta_dot_traj,
            self.boost_traj,
            self.pitch_traj,
        ) = self.interpolate_trajectory(
            t_traj=t_traj,
            y_traj=y_traj,
            z_traj=z_traj,
            y_dot_traj=y_dot_traj,
            z_dot_traj=z_dot_traj,
            theta_traj=theta_traj,
            theta_dot_traj=theta_dot_traj,
            boost_traj=boost_traj,
            pitch_traj=pitch_traj,
        )
        # print(self.z_dot_traj)
        # print(self.pitch_traj)
        self.count = 0

        # For saving
        self.y_history = []
        self.z_history = []
        self.y_dot_history = []
        self.z_dot_history = []
        self.theta_history = []
        self.theta_dot_history = []
        self.boost_history = []
        self.pitch_history = []

        self.y_traj_history = []
        self.z_traj_history = []
        self.y_dot_traj_history = []
        self.z_dot_traj_history = []
        self.theta_traj_history = []
        self.theta_dot_traj_history = []
        self.boost_traj_history = []
        self.pitch_traj_history = []

        self.t_history = []

        self.theta_des_history = []

        self.a_traj_y_history = []
        self.a_traj_z_history = []
        self.a_lat_y_history = []
        self.a_lat_z_history = []
        self.a_des_y_history = []
        self.a_des_z_history = []
        self.boost_des_history = []

        self.is_saved = False

        self.energy = 0
        self.energy_prev = 0

    def set_state(self, q: np.array) -> CarState:
        """
        Sets the bot's state

        Args:
            q (np.array): new bot state
        """
        self.q.update_with_array(state_array=q)
        self.model.q = copy.deepcopy(self.q)
        phi, theta, psi = lau.quaternion_to_euler(
            e0=self.q.e0, e1=self.q.e1, e2=self.q.e2, e3=self.q.e3
        )
        psi = np.pi / 2
        # theta = np.pi / 2
        theta = 0  # np.pi / 2  # np.pi / 2
        # if theta > np.pi / 2:
        #    theta = np.pi - theta

        car_state = CarState(
            boost_amount=100,
            physics=Physics(
                # location=Vector3(self.q.x, self.q.y, self.q.z),  # world frame
                location=Vector3(0, -1500, 600),  # world frame
                velocity=Vector3(self.q.x_dot, self.q.y_dot, 0),  # world frame
                rotation=Rotator(theta, psi, phi),
                angular_velocity=Vector3(
                    self.q.phi_dot, self.q.theta_dot, self.q.psi_dot
                ),  # world frame
            ),
        )
        return car_state

    def step(self, quat_des: np.array, theta) -> SimpleControllerState:
        controls = SimpleControllerState()
        pos = self.q()[:3]
        v = self.q()[3:6]
        quat = Quaternion(real=self.q()[9], imaginary=self.q()[6:9])
        omega = self.q()[10:]

        # if self.count < self.num_timesteps:
        #     # TODO: handle this externally (geometric controller class?)
        #     v_traj = np.array(
        #         [self.y_dot_traj[self.count], self.z_dot_traj[self.count]]
        #     )  # trajectory velocity at timestep: self.count [uu/s]
        #     v_yz = v[1:]  # yz-velocity [uu/s]
        #     e_velocity = v_traj - v_yz  # velocity error vector

        #     P_y_dot = 1
        #     P_z_dot = 1
        #     k = np.array([P_y_dot, P_z_dot])
        #     g_feedforward = np.array([0, 660])  # feedforward term for gravity
        #     v_des = k * e_velocity + g_feedforward * self.dt  # desired velocity vector

        #     y_axis = np.array([1, 0])  # y-axis vector
        #     theta_des = np.arccos(
        #         np.dot(
        #             v_des / np.linalg.norm(v_des),
        #             y_axis / np.linalg.norm(y_axis),
        #         )
        #     )
        #     direction = np.sign(
        #         np.arcsin(
        #             np.cross(
        #                 y_axis / np.linalg.norm(y_axis),
        #                 v_des / np.linalg.norm(v_des),
        #             )
        #         )
        #     )
        #     theta_des *= direction

        #     omega_des = self.pid_theta.step(desired=theta_des, actual=0)
        #     u_pitch = self.pid_omega.step(desired=omega_des, actual=omega[1])
        #     if self.boost_traj[self.count] > 0:
        #         boost = 1
        #     else:
        #         boost = 0

        #     pitch = (u_pitch) / 12.14599781908070
        #     # pitch = self.pitch_traj[self.count]

        #     controls.pitch = np.clip(pitch, -1, 1)
        #     # controls.pitch = pitch
        #     # print(controls.pitch)
        #     # controls.pitch = -1
        #     controls.boost = boost
        # self.t += self.dt
        # self.count += 1
        # return controls

        # boost = self.bang_bang.step(quat=quat, v=v, v_des=self.v_des)

        # v_des = np.array([self.y_dot_traj[counter], self.z_dot_traj[counter]]) # desired velocity to follow trajectory
        try:
            # l = 100  # lookahead distance [uu]
            # y = pos[1]  # bot's y-position [uu]
            # z = pos[2]  # bot's z-position [uu]
            # y_traj = self.y_traj[self.count :]
            # z_traj = self.z_traj[self.count :]

            # # Find look-ahead intersection
            # dy = [y - y_t for y_t in y_traj]
            # dz = [z - z_t for z_t in z_traj]
            # d = np.hypot(dy, dz)
            # target_index = np.argmin(np.abs(l - d))  # target point index
            # pos_target = np.array(
            #     [y_traj[target_index], z_traj[target_index]]
            # )  # target point
            # # print(f"target point: {pos_target}")
            # # Find vector from bot to target point
            # pos_yz = np.array([y, z])  # yz-position
            # l_vector = pos_target - pos_yz  # look-ahead vector
            # v_yz = v[1:]
            # theta_d = np.arccos(
            #     np.dot(
            #         v_yz / np.linalg.norm(v_yz), pos_target / np.linalg.norm(pos_target)
            #     )
            # )
            # direction = np.sign(
            #     np.arcsin(
            #         np.cross(
            #             pos_target / np.linalg.norm(pos_target),
            #             v_yz / np.linalg.norm(v_yz),
            #         )
            #     )
            # )
            # theta_d *= direction

            # dot_product = np.dot(
            #     pos_yz / np.linalg.norm(pos_yz), pos_target / np.linalg.norm(pos_target)
            # )
            # alpha = np.arccos(dot_product)  # error angle
            """
            l_target = pos_target / np.linalg.norm(
                pos_target
            )  # approximate system look-ahead distance [uu]
            k = 2 * np.sin(alpha) / l_target  # curvature
            """

            # omega_des = self.pid_theta.step(desired=theta_d, actual=0)
            # u_pitch = self.pid_omega.step(desired=omega_des, actual=omega[1])

            # boost = 1

            a_traj = np.array(
                [
                    self.boost_traj[self.count] * np.cos(self.theta_traj[self.count]),
                    self.boost_traj[self.count] * np.sin(self.theta_traj[self.count]),
                ]
            )

            # ---------- REMOVE ----------#
            self.a_traj_y_history.append(a_traj[0])
            self.a_traj_z_history.append(a_traj[1])

            a_lat = np.array(
                [self.y_traj[self.count] - pos[1], self.z_traj[self.count] - pos[2]]
            )

            # ---------- REMOVE ----------#
            self.a_lat_y_history.append(a_lat[0])
            self.a_lat_z_history.append(a_lat[1])

            a_lat_norm = (a_lat[0] ** 2 + a_lat[1] ** 2) ** (1 / 2)
            # if a_lat_norm < 100:
            #     k_p = 0
            # else:
            #     k_p = 2

            # k_p = np.array([0, -7.5])  # -0.75
            k_p = 2
            a_des = a_traj + k_p * a_lat

            # ---------- REMOVE ----------#
            self.a_des_y_history.append(a_des[0])
            self.a_des_z_history.append(a_des[1])

            boost_des = (a_des[0] ** 2 + a_des[1] ** 2) ** (1 / 2)

            # ---------- REMOVE ----------#
            self.boost_des_history.append(boost_des)

            print(f"error: {a_lat} | traj: {a_traj} | desired: {a_des}")
            x_axis = np.array([1, 0, 0])
            vector = quat.rotate(x_axis)[1:]
            vector[1] *= -1
            theta_des = self.angle_between_vectors(vect=a_des, vect_des=x_axis[:2])
            theta = self.angle_between_vectors(vect=vector, vect_des=x_axis[:2])

            # vect_des = np.array(
            #     [
            #         np.cos(self.theta_traj[self.count]),
            #         np.sin(self.theta_traj[self.count]),
            #     ]
            # )
            # # vect_des = np.array([1, 1])
            # x_axis = np.array([1, 0, 0])
            # vector = quat.rotate(x_axis)[1:]
            # vector[1] *= -1
            # theta_des = self.angle_between_vectors(vect=vect_des, vect_des=x_axis[:2])
            # # print(f"theta_des: {theta_des}")

            # theta = self.angle_between_vectors(vect=vector, vect_des=x_axis[:2])

            # u_pitch = self.pid_theta.step(desired=theta_des, actual=theta)

            omega_des = self.pid_theta.step(desired=theta_des, actual=theta)
            omega_des += 0.75 * self.theta_dot_traj[self.count]
            # print(omega_des)
            u_pitch = self.pid_omega.step(desired=omega_des, actual=-omega[1])
            # print(
            #     f"vector: {vector} | theta: {theta} | theta_des: {theta_des} | input: {u_pitch}"
            # )
            # print(u_pitch)
            pitch = (u_pitch + 0.5 * self.pitch_traj[self.count]) / 12.14599781908070
            # pitch = u_pitch / 12.14599781908070
            # pitch = self.pitch_traj[self.count] / 12.14599781908070
            if np.abs(pitch) > 1:
                # print(f"Pitch input out of bounds: {pitch}")
                pass
            # pitch = 1

            controls.pitch = np.clip(pitch, -1, 1)

            if self.energy_prev - 991.666 * self.dt >= 0:
                boost = 1
            else:
                boost = 0

            # self.energy = self.energy_prev + (
            #     self.boost_traj[self.count] * self.dt - boost * 991.666 * self.dt
            # )

            self.energy = self.energy_prev + (
                boost_des * self.dt - boost * 991.666 * self.dt
            )

            self.energy_prev = self.energy

            controls.boost = boost
            # boost = self.bang_bang.step(quat=quat, v=v, v_des=self.v_des)

            self.y_history.append(self.q()[1])
            self.z_history.append(self.q()[2])
            self.y_dot_history.append(self.q()[4])
            self.z_dot_history.append(self.q()[5])
            self.theta_history.append(theta * 180 / np.pi)
            self.theta_dot_history.append(self.q()[11] * 180 / np.pi)
            self.boost_history.append(boost * 991.666)
            self.pitch_history.append(pitch)

            self.y_traj_history.append(self.y_traj[self.count])
            self.z_traj_history.append(self.z_traj[self.count])
            self.y_dot_traj_history.append(self.y_dot_traj[self.count])
            self.z_dot_traj_history.append(self.z_dot_traj[self.count])
            self.theta_traj_history.append(self.theta_traj[self.count] * 180 / np.pi)
            self.theta_dot_traj_history.append(self.theta_dot_traj[self.count])
            self.boost_traj_history.append(self.boost_traj[self.count])
            self.pitch_traj_history.append(
                self.pitch_traj[self.count] / 12.14599781908070
            )
            self.t_history.append(self.t)

            self.theta_des_history.append(theta_des)
            # self.theta_des_history.append(1)

        except Exception as e:
            # print(e)
            pass

        if self.t > self.t_traj[-1]:
            if self.is_saved == False:
                print("saving")
                self.save_data()
                self.is_saved = True

        self.t += self.dt
        self.count += 1
        return controls

    # theta_des = np.pi / 4  # [rads]
    # self.t += self.dt
    # # theta_des = np.pi / 4  # [rads]
    # omega_des = self.pid_theta.step(desired=theta_des, actual=theta)
    # # omega_des = self.pid_theta.step(desired=self.theta_des, actual=theta)
    # u_pitch = self.pid_omega.step(desired=omega_des, actual=omega[1])

    # controls.pitch = 0
    # controls.boost = 0  # boost

    # ---------- Full State Feedback ---------- #
    # TODO: Reimplement for 3D cases
    # v_des = np.array([0, 1000, 0])
    # quat_des = self.geometric_controller.step(v=v, v_des=v_des, quat=quat)
    # boost = self.bang_bang.step(quat=quat, v=v, v_des=v_des)
    # tau = self.p2_cascade.step(quat=quat, quat_des=quat_des, omega=omega)
    # controls.roll = tau[0] / 36.07956616966136
    # controls.pitch = tau[1] / 12.14599781908070  # TODO: Check sign
    # controls.yaw = tau[2] / 8.91962804287785
    # controls.roll = np.clip(controls.roll, -1, 1)
    # controls.pitch = np.clip(controls.pitch, -1, 1)
    # controls.yaw = np.clip(controls.yaw, -1, 1)
    # # boost = 0
    # controls.boost = boost
    # u = np.array([controls.roll, controls.pitch, controls.yaw, controls.boost])
    # self.model.step(u=u)

    def angle_between_vectors(self, vect: np.array, vect_des: np.array):
        theta_des = np.arccos(
            np.dot(vect / np.linalg.norm(vect), vect_des / np.linalg.norm(vect_des))
        )
        direction = np.sign(
            np.arcsin(
                np.cross(
                    vect_des / np.linalg.norm(vect_des),
                    vect / np.linalg.norm(vect),
                )
            )
        )
        theta_des *= direction
        return theta_des

    def save_data(self):
        """
        TODO
        """
        save_list = [
            self.y_history,
            self.z_history,
            self.y_dot_history,
            self.z_dot_history,
            self.theta_history,
            self.theta_dot_history,
            self.boost_history,
            self.pitch_history,
            self.y_traj_history,
            self.z_traj_history,
            self.y_dot_traj_history,
            self.z_dot_traj_history,
            self.theta_traj_history,
            self.theta_dot_traj_history,
            self.boost_traj_history,
            self.pitch_traj_history,
            self.t_history,
            self.theta_des_history,
            self.a_traj_y_history,
            self.a_traj_z_history,
            self.a_lat_y_history,
            self.a_lat_z_history,
            self.a_des_y_history,
            self.a_des_z_history,
            self.boost_des_history,
        ]
        np.savetxt(
            "C:/Users/Student/Documents/RLBot_IS/trajectory-optimization/data/bot_data.csv",
            save_list,
            delimiter=", ",
            fmt="% s",
        )

    def load_trajectory(self) -> List[float]:
        """
        TODO
        """
        trajectory_data = self.history.load_tj()
        input_data = np.genfromtxt(
            "C:/Users/Student/Documents/RLBot_IS/trajectory-optimization/data/input_data.csv",
            delimiter=",",
        )

        t_traj = trajectory_data[0]
        y_traj = trajectory_data[1]
        z_traj = trajectory_data[2]
        y_dot_traj = trajectory_data[3]
        z_dot_traj = trajectory_data[4]
        theta_traj = trajectory_data[5]
        theta_dot_traj = trajectory_data[6]

        boost_traj = np.empty(
            t_traj.shape,
        )
        boost_traj[:-1] = input_data[0]
        boost_traj[-1] = 0
        pitch_traj = np.empty(
            t_traj.shape,
        )
        pitch_traj[:-1] = input_data[1]
        pitch_traj[-1] = 0

        return [
            t_traj,
            y_traj,
            z_traj,
            y_dot_traj,
            z_dot_traj,
            theta_traj,
            theta_dot_traj,
            boost_traj,
            pitch_traj,
        ]

    def interpolate_trajectory(
        self,
        t_traj: float,
        y_traj: float,
        z_traj: float,
        y_dot_traj: float,
        z_dot_traj: float,
        theta_traj: float,
        theta_dot_traj: float,
        boost_traj: float,
        pitch_traj: float,
    ):
        """
        TODO:
        """
        print(pitch_traj)
        t_traj_f = t_traj[-1]  # final time of optimized trajectory [s]
        num_timesteps = int(
            t_traj_f / self.dt
        )  # number of timesteps due to simulation control loop
        t = np.linspace(
            0, num_timesteps * self.dt, num_timesteps
        )  # control loop times for trajectory following

        j = 0  # counter for trajectory timesteps
        y = np.empty(
            num_timesteps,
        )  # interpolated y-position
        z = np.empty(
            num_timesteps,
        )  # interpolated z-position
        y_dot = np.empty(
            num_timesteps,
        )  # interpolated y-velocity
        z_dot = np.empty(
            num_timesteps,
        )  # interpolated z-velocity
        theta = np.empty(
            num_timesteps,
        )  # interpolated pitch
        theta_dot = np.empty(
            num_timesteps,
        )  # interpolated pitch rate
        boost = np.empty(
            num_timesteps,
        )  # interpolated boost input
        pitch = np.empty(
            num_timesteps,
        )  # interpolated pitch input
        for i in range(num_timesteps):
            if t[i] == t_traj[j]:
                y[i] = y_traj[j]
                z[i] = z_traj[j]
                y_dot[i] = y_dot_traj[j]
                z_dot[i] = z_dot_traj[j]
                theta[i] = theta_traj[j]
                theta_dot[i] = theta_dot_traj[j]
                boost[i] = boost_traj[j]
                pitch[i] = pitch_traj[j]
            elif t[i] == t_traj[j + 1]:
                y[i] = y_traj[j + 1]
                z[i] = z_traj[j + 1]
                y_dot[i] = y_dot_traj[j + 1]
                z_dot[i] = z_dot_traj[j + 1]
                theta[i] = theta_traj[j + 1]
                theta_dot[i] = theta_dot_traj[j + 1]
                boost[i] = boost_traj[j + 1]
                pitch[i] = pitch_traj[j + 1]
                j += 1
            elif t[i] > t_traj[j + 1]:
                y[i] = (y_traj[j + 2] - y_traj[j + 1]) / (
                    t_traj[j + 2] - t_traj[j + 1]
                ) * (t[i] - t_traj[j + 1]) + y_traj[j + 1]
                z[i] = (z_traj[j + 2] - z_traj[j + 1]) / (
                    t_traj[j + 2] - t_traj[j + 1]
                ) * (t[i] - t_traj[j + 1]) + z_traj[j + 1]
                y_dot[i] = (y_dot_traj[j + 2] - y_dot_traj[j + 1]) / (
                    t_traj[j + 2] - t_traj[j + 1]
                ) * (t[i] - t_traj[j + 1]) + y_dot_traj[j + 1]
                z_dot[i] = (z_dot_traj[j + 2] - z_dot_traj[j + 1]) / (
                    t_traj[j + 2] - t_traj[j + 1]
                ) * (t[i] - t_traj[j + 1]) + z_dot_traj[j + 1]
                theta[i] = (theta_traj[j + 2] - theta_traj[j + 1]) / (
                    t_traj[j + 2] - t_traj[j + 1]
                ) * (t[i] - t_traj[j + 1]) + theta_traj[j + 1]
                theta_dot[i] = (theta_dot_traj[j + 2] - theta_dot_traj[j + 1]) / (
                    t_traj[j + 2] - t_traj[j + 1]
                ) * (t[i] - t_traj[j + 1]) + theta_dot_traj[j + 1]
                boost[i] = (boost_traj[j + 2] - boost_traj[j + 1]) / (
                    t_traj[j + 2] - t_traj[j + 1]
                ) * (t[i] - t_traj[j + 1]) + boost_traj[j + 1]
                pitch[i] = (pitch_traj[j + 2] - pitch_traj[j + 1]) / (
                    t_traj[j + 2] - t_traj[j + 1]
                ) * (t[i] - t_traj[j + 1]) + pitch_traj[j + 1]
                j += 1
            elif t[i] > t_traj[j]:
                y[i] = (y_traj[j + 1] - y_traj[j]) / (t_traj[j + 1] - t_traj[j]) * (
                    t[i] - t_traj[j]
                ) + y_traj[j]
                z[i] = (z_traj[j + 1] - z_traj[j]) / (t_traj[j + 1] - t_traj[j]) * (
                    t[i] - t_traj[j]
                ) + z_traj[j]
                y_dot[i] = (y_dot_traj[j + 1] - y_dot_traj[j]) / (
                    t_traj[j + 1] - t_traj[j]
                ) * (t[i] - t_traj[j]) + y_dot_traj[j]
                z_dot[i] = (z_dot_traj[j + 1] - z_dot_traj[j]) / (
                    t_traj[j + 1] - t_traj[j]
                ) * (t[i] - t_traj[j]) + z_dot_traj[j]
                theta[i] = (theta_traj[j + 1] - theta_traj[j]) / (
                    t_traj[j + 1] - t_traj[j]
                ) * (t[i] - t_traj[j]) + theta_traj[j]
                theta_dot[i] = (theta_dot_traj[j + 1] - theta_dot_traj[j]) / (
                    t_traj[j + 1] - t_traj[j]
                ) * (t[i] - t_traj[j]) + theta_dot_traj[j]
                boost[i] = (boost_traj[j + 1] - boost_traj[j]) / (
                    t_traj[j + 1] - t_traj[j]
                ) * (t[i] - t_traj[j]) + boost_traj[j]
                pitch[i] = (pitch_traj[j + 1] - pitch_traj[j]) / (
                    t_traj[j + 1] - t_traj[j]
                ) * (t[i] - t_traj[j]) + pitch_traj[j]
        # print(pitch)
        return [
            num_timesteps,
            t,
            y,
            z,
            y_dot,
            z_dot,
            theta,
            theta_dot,
            boost,
            pitch,
        ]


if __name__ == "__main__":

    bot = Bot()
    print(bot.num_timesteps)
    import matplotlib.pyplot as plt

    plt.figure()
    plt.plot(bot.t_traj, bot.pitch_traj)
    plt.show()

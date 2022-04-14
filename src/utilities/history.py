import numpy as np


class History:
    def __init__(self):
        self.t_history = []  # time history of simulation

        self.x_history = []  # x-position history of bot [uu]
        self.y_history = []  # y-position history of bot [uu]
        self.z_history = []  # z-position history of bot [uu]
        self.x_dot_history = []  # x-velocity history of bot [uu/s]
        self.y_dot_history = []  # y-velocity history of bot [uu/s]
        self.z_dot_history = []  # z-velocity history of bot [uu/s]
        self.e0_history = []  # x-quaternion history of bot
        self.e1_history = []  # y-quaternion history of bot
        self.e2_history = []  # z-quaternion history of bot
        self.e3_history = []  # w-quaternion history of bot
        self.phi_dot_history = []  # roll rate history of bot [degrees/s]
        self.theta_dot_history = []  # pitch rate history of bot [degrees/s]
        self.psi_dot_history = []  # yaw rate history of bot [degrees/s]

        self.x_m_history = []  # x-position history of model [uu]
        self.y_m_history = []  # y-position history of model [uu]
        self.z_m_history = []  # z-position history of model [uu]
        self.x_dot_m_history = []  # x-velocity history of model [uu/s]
        self.y_dot_m_history = []  # y-velocity history of model [uu/s]
        self.z_dot_m_history = []  # z-velocity history of model [uu/s]
        self.e0_m_history = []  # x-quaternion history of model
        self.e1_m_history = []  # y-quaternion history of model
        self.e2_m_history = []  # z-quaternion history of model
        self.e3_m_history = []  # w-quaternion history of model
        self.phi_dot_m_history = []  # roll rate history of model [degrees/s]
        self.theta_dot_m_history = []  # pitch rate history of model [degrees/s]
        self.psi_dot_m_history = []  # yaw rate history of model [degrees/s]

    def append_many(
        self,
        t: float,
        x: float = None,
        y: float = None,
        z: float = None,
        x_dot: float = None,
        y_dot: float = None,
        z_dot: float = None,
        e0: float = None,
        e1: float = None,
        e2: float = None,
        e3: float = None,
        phi_dot: float = None,
        theta_dot: float = None,
        psi_dot: float = None,
        x_m: float = None,
        y_m: float = None,
        z_m: float = None,
        x_dot_m: float = None,
        y_dot_m: float = None,
        z_dot_m: float = None,
        e0_m: float = None,
        e1_m: float = None,
        e2_m: float = None,
        e3_m: float = None,
        phi_dot_m: float = None,
        theta_dot_m: float = None,
        psi_dot_m: float = None,
    ):
        """
        Appends all simulation variables at once

        Args:
            t (float): time [s]
            x (float): x-position of bot [uu]
            y (float): y-position of bot [uu]
            z (float): z-position of zbot [uu]
            x_dot (float): x-velocity of bot [uu]
            y_dot (float): y-velocity of bot [uu]
            z_dot (float): z-position of bot [uu]
            e0 (float): x-quaternion of bot
            e1 (float): y-quaternion of bot
            e2 (float): z-quaternion of bot
            e3 (float): w-quaternion of bot
            phi_dot (float): roll rate of bot [rad/s]
            theta_dot (float): pitch rate of bot [rad/s]
            psi_dot (float): yaw rate of bot [rad/s]
            x_m (float): x-position of model [uu]
            y_m (float): y-position of model [uu]
            z_m (float): z-position of model [uu]
            x_dot_m (float): x-velocity of model [uu]
            y_dot_m (float): y-velocity of model [uu]
            z_dot_m (float): z-position of model [uu]
            e0_m (float): x-quaternion of model
            e1_m (float): y-quaternion of model
            e2_m (float): z-quaternion of model
            e3_m (float): w-quaternion of model
            phi_dot_m (float): roll rate of model [rad/s]
            theta_dot_m (float): pitch rate of model [rad/s]
            psi_dot_m (float): yaw rate of model [rad/s]
        """
        # TODO: If (none) append
        self.t_history.append(t)

        self.x_history.append(x)
        self.y_history.append(y)
        self.z_history.append(z)
        self.x_dot_history.append(x_dot)
        self.y_dot_history.append(y_dot)
        self.z_dot_history.append(z_dot)
        self.e0_history.append(e0)
        self.e1_history.append(e1)
        self.e2_history.append(e2)
        self.e3_history.append(e3)
        self.phi_dot_history.append(phi_dot * 180 / np.pi)
        self.theta_dot_history.append(theta_dot * 180 / np.pi)
        self.psi_dot_history.append(psi_dot * 180 / np.pi)

        self.x_m_history.append(x_m)
        self.y_m_history.append(y_m)
        self.z_m_history.append(z_m)
        self.x_dot_m_history.append(x_m_dot)
        self.y_dot_m_history.append(y_m_dot)
        self.z_dot_m_history.append(z_m_dot)
        self.e0_m_history.append(e0_m)
        self.e1_m_history.append(e1_m)
        self.e2_m_history.append(e2_m)
        self.e3_m_history.append(e3_m)
        self.phi_dot_m_history.append(phi_dot_m * 180 / np.pi)
        self.theta_dot_m_history.append(theta_dot_m * 180 / np.pi)
        self.psi_dot_m_history.append(psi_dot_m * 180 / np.pi)

    def append_many_with_array(self, t: float, q: np.array, q_m: np.array):
        """
        Appends all simulation variables at once using arrays

        Args:
            t (float): time [s]
            q (np.array): bot's state
            q_m (np.array): model's state
        """
        self.t_history.append(t)

        self.x_history.append(q[0])
        self.y_history.append(q[1])
        self.z_history.append(q[2])
        self.x_dot_history.append(q[3])
        self.y_dot_history.append(q[4])
        self.z_dot_history.append(q[5])
        self.e0_history.append(q[6])
        self.e1_history.append(q[7])
        self.e2_history.append(q[8])
        self.e3_history.append(q[9])
        self.phi_dot_history.append(q[10] * 180 / np.pi)
        self.theta_dot_history.append(q[11] * 180 / np.pi)
        self.psi_dot_history.append(q[12] * 180 / np.pi)

        self.x_m_history.append(q_m[0])
        self.y_m_history.append(q_m[1])
        self.z_m_history.append(q_m[2])
        self.x_dot_m_history.append(q_m[3])
        self.y_dot_m_history.append(q_m[4])
        self.z_dot_m_history.append(q_m[5])
        self.e0_m_history.append(q_m[6])
        self.e1_m_history.append(q_m[7])
        self.e2_m_history.append(q_m[8])
        self.e3_m_history.append(q_m[9])
        self.phi_dot_m_history.append(q_m[10] * 180 / np.pi)
        self.theta_dot_m_history.append(q_m[11] * 180 / np.pi)
        self.psi_dot_m_history.append(q_m[12] * 180 / np.pi)

        def append_many_trajectory(self, t: float, x: np.array, u: np.array):
            """
            Appends optimal path and and inputs

            Args:
                t (float): time [s]
                q (np.array): bot's state
                q_m (np.array): model's state
            """
            self.t_history.append(t)

            self.x_history.append(q[0])
            self.y_history.append(q[1])
            self.z_history.append(q[2])
            self.x_dot_history.append(q[3])
            self.y_dot_history.append(q[4])
            self.z_dot_history.append(q[5])
            self.e0_history.append(q[6])
            self.e1_history.append(q[7])
            self.e2_history.append(q[8])
            self.e3_history.append(q[9])
            self.phi_dot_history.append(q[10] * 180 / np.pi)
            self.theta_dot_history.append(q[11] * 180 / np.pi)
            self.psi_dot_history.append(q[12] * 180 / np.pi)

            self.Ux_history.append(u[0])
            self.Uphi_history.append(u[1])
            self.Utheta_history.append(u[2])
            self.Upsi_history.append(u[3])

    def save(self):
        """
        Saves history data to csv file

        Returns:
            True
        """
        save_list = [
            self.t_history,
            self.x_history,
            self.y_history,
            self.z_history,
            self.x_dot_history,
            self.y_dot_history,
            self.z_dot_history,
            self.e0_history,
            self.e1_history,
            self.e2_history,
            self.e3_history,
            self.phi_dot_history,
            self.theta_dot_history,
            self.psi_dot_history,
            self.x_m_history,
            self.y_m_history,
            self.z_m_history,
            self.x_dot_m_history,
            self.y_dot_m_history,
            self.z_dot_m_history,
            self.e0_m_history,
            self.e1_m_history,
            self.e2_m_history,
            self.e3_m_history,
            self.phi_dot_m_history,
            self.theta_dot_m_history,
            self.psi_dot_m_history,
        ]
        np.savetxt(
            "C:/Users/Student/Documents/RLBot_IS/trajectory-optimization/data/data.csv",
            save_list,
            delimiter=", ",
            fmt="% s",
        )

        return True

    def load(self):
        """
        Loads history data from csv file

        Returns:
            load_data (List): all history data originally saved to csv
        """
        load_data = np.genfromtxt(
            "C:/Users/Student/Documents/RLBot_IS/trajectory-optimization/data/data.csv",
            delimiter=",",
        )

        return load_data

import numpy as np
import copy


class LinAlgUtils:
    @staticmethod
    def euler_to_quaternion(phi: float, theta: float, psi: float) -> list:
        """
        Converts Euler angles to quaternions

        Args:
            phi (float): The roll (rotation around x-axis) [rad]
            theta (float): The pitch (rotation around y-axis) [rad]
            psi (float): The yaw (rotation around z-axis) [rad]

        Returns:
            [e0, e1, e2, e3] (list): List of quaternions
        """
        e0 = np.sin(phi / 2) * np.cos(theta / 2) * np.cos(psi / 2) - np.cos(
            phi / 2
        ) * np.sin(theta / 2) * np.sin(psi / 2)
        e1 = np.cos(phi / 2) * np.sin(theta / 2) * np.cos(psi / 2) + np.sin(
            phi / 2
        ) * np.cos(theta / 2) * np.sin(psi / 2)
        e2 = np.cos(phi / 2) * np.cos(theta / 2) * np.sin(psi / 2) - np.sin(
            phi / 2
        ) * np.sin(theta / 2) * np.cos(psi / 2)
        e3 = np.cos(phi / 2) * np.cos(theta / 2) * np.cos(psi / 2) + np.sin(
            phi / 2
        ) * np.sin(theta / 2) * np.sin(psi / 2)

        return [e0, e1, e2, e3]

    @staticmethod
    def quaternion_to_euler(e0: float, e1: float, e2: float, e3: float) -> list:
        """
        Converts quaternions into Euler angles

        Args:
            e0 (float): x-quaternion
            e1 (float): y-quaternion
            e2 (float): z-quaternion
            e3 (float): w-quaternion

        Returns:
            [phi, theta, psi] (list): Roll, pitch, and yaw of system [rad]
        """
        t0 = +2.0 * (e3 * e0 + e1 * e2)
        t1 = +1.0 - 2.0 * (e0 * e0 + e1 * e1)
        phi = np.arctan2(t0, t1)

        t2 = +2.0 * (e3 * e1 - e2 * e0)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        theta = np.arcsin(t2)

        t3 = +2.0 * (e3 * e2 + e0 * e1)
        t4 = +1.0 - 2.0 * (e1 * e1 + e2 * e2)
        psi = np.arctan2(t3, t4)

        return [phi, theta, psi]

    @staticmethod
    def quaternion_to_rotation(e0: float, e1: float, e2: float, e3: float) -> np.array:
        """
        Converts quaternion into rotation matrix

        Args:
            e0 (float): x-quaternion
            e1 (float): y-quaternion
            e2 (float): z-quaternion
            e3 (float): w-quaternion

        Returns:
            R (np.array): rotation matrix
        """
        r_00 = 2 * (e0 * e0 + e1 * e1) - 1
        r_01 = 2 * (e1 * e2 - e0 * e3)
        r_02 = 2 * (e1 * e3 + e0 * e2)
        r_10 = 2 * (e1 * e2 + e0 * e3)
        r_11 = 2 * (e0 * e0 + e2 * e2) - 1
        r_12 = 2 * (e2 * e3 - e0 * e1)
        r_20 = 2 * (e1 * e3 - e0 * e2)
        r_21 = 2 * (e2 * e3 + e0 * e1)
        r_22 = 2 * (e0 * e0 + e3 * e3) - 1
        R = np.array([[r_00, r_01, r_02], [r_10, r_11, r_12], [r_20, r_21, r_22]])

        return R

    @staticmethod
    def quat_multiply(quat_0: np.array, quat_1: np.array) -> np.array:
        """
        Multiples two quaternions

        Args:
            quat_0 (np.array): first quaternion
            quat_1 (np.array): second quaternion

        Returns:
            quat_2 (np.array): multplied quaternion
        """
        x_0 = quat_0[0]
        y_0 = quat_0[1]
        z_0 = quat_0[2]
        w_0 = quat_0[3]
        x_1 = quat_1[0]
        y_1 = quat_1[1]
        z_1 = quat_1[2]
        w_1 = quat_1[3]

        w_2 = w_0 * w_1 - x_0 * x_1 - y_0 * y_1 - z_0 * z_1
        x_2 = w_0 * x_1 + x_0 * w_1 + y_0 * z_1 - z_0 * y_1
        y_2 = w_0 * y_1 + y_0 * w_1 + z_0 * x_1 - x_0 * z_1
        z_2 = w_0 * z_1 + z_0 * w_1 + x_0 * y_1 - y_0 * x_1

        quat_2 = np.array([x_2, y_2, z_2, w_2])

        return quat_2

    @staticmethod
    def quat_conjugate(quat: np.array) -> np.array:
        """
        Gives the conjugate of a quaternion

        Args:
            quat (np.array): quaternion

        Returns:
            quat_conj (np.array): conjugate quaternion
        """
        quat_conj = np.empty((4,))
        quat_conj[0:3] = -1 * copy.deepcopy(quat[0:3])
        quat_conj[3] = copy.deepcopy(quat[3])
        return quat_conj

    @staticmethod
    def quat_normalize(quat: np.array) -> np.array:
        """
        Create a unit quaternion

        Args:
            quat (np.array): quaternion

        Returns:
            quat_unit (np.array): unit quaternion
        """
        quat_unit = quat / np.linalg.norm(quat)

        return quat_unit

    @staticmethod
    def quat_body_to_world(quat_0: np.array, quat_1: np.array) -> np.array:
        """
        Converts quaternion from body frame to world frame

        Args:
            quat_0 (np.array): quaternion
            quat_1 (np.array): quaternion

        Returns:
            quat (np.array): unit quaternion
        """

        quat_0_conj = LinAlgUtils.quat_conjugate(quat_0)
        rhs = LinAlgUtils.quat_multiply(quat_1, quat_0_conj)  # right hand side
        quat = LinAlgUtils.quat_multiply(quat_0, rhs)
        return quat

    @staticmethod
    def quat_world_to_body(quat_0: np.array, quat_1: np.array) -> np.array:
        """
        Converts quaternion from body frame to world frame

        Args:
            quat_0 (np.array): quaternion
            quat_1 (np.array): quaternion

        Returns:
            quat (np.array): unit quaternion
        """
        quat_0_conj = LinAlgUtils.quat_conjugate(quat_0)
        rhs = LinAlgUtils.quat_multiply(quat_0=quat_1, quat_1=quat_0)  # right hand side
        quat = LinAlgUtils.quat_multiply(quat_0_conj, rhs)
        return quat

    @staticmethod
    def quat_rotation_between_two_vectors(vect_0: np.array, vect_1: np.array):
        """
        TODO
        """
        k_cos_theta = np.dot(vect_0, vect_1)
        k = (np.linalg.norm(vect_0) ** 2 * np.linalg.norm(vect_1) ** 2) ** (1 / 2)
        if k_cos_theta / k == -1:
            R = np.array(
                [
                    [np.cos(np.pi / 2), 0, -np.sin(np.pi / 2)],
                    [0, 1, 0],
                    [-np.sin(np.pi / 2), 0, np.cos(np.pi / 2)],
                ]
            )
            vect_orth = R @ vect_0
            print(vect_orth)
            quat = np.array([vect_orth[0], vect_orth[1], vect_orth[2], 0])
            return quat
        vect_cross = np.cross(vect_0, vect_1)
        quat = np.array([vect_cross[0], vect_cross[1], vect_cross[2], k_cos_theta + k])
        quat = LinAlgUtils.quat_normalize(quat=quat)
        return quat


if __name__ == "__main__":
    u = np.array([1, 0, 0])
    v = np.array([500, 500, 0])
    w = LinAlgUtils.quat_rotation_between_two_vectors(u, v)
    print(LinAlgUtils.quat_normalize(w))

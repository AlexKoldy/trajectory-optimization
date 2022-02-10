import numpy as np


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
    phi = np.atan2(t0, t1)

    t2 = +2.0 * (e3 * e1 - e2 * e0)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    theta = np.asin(t2)

    t3 = +2.0 * (e3 * e2 + e0 * e1)
    t4 = +1.0 - 2.0 * (e1 * e1 + e2 * e2)
    psi = np.atan2(t3, t4)

    return [phi, theta, psi]


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

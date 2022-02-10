import numpy as np

def euler_to_quaternion(phi: float, theta: float, psi: float):
    """
    Converts Euler angles to quaternions

    Args:
        roll (float): The roll (rotation around x-axis) [rad]
        pitch (float): The pitch (rotation around y-axis) [rad]
        yaw (float): The yaw (rotation around z-axis) [rad]

    Returns:
        [e0, e1, e2, e3]: List of quaternions
    """

    e0 = np.sin(phi/2) * np.cos(theta/2) * np.cos(yaw/2) - np.cos(phi/2) * np.sin(theta/2) * np.sin(yaw/2)
    e1 = np.cos(phi/2) * np.sin(theta/2) * np.cos(yaw/2) + np.sin(phi/2) * np.cos(theta/2) * np.sin(yaw/2)
    e2 = np.cos(phi/2) * np.cos(theta/2) * np.sin(yaw/2) - np.sin(phi/2) * np.sin(theta/2) * np.cos(yaw/2)
    e3 = np.cos(phi/2) * np.cos(theta/2) * np.cos(yaw/2) + np.sin(phi/2) * np.sin(theta/2) * np.sin(yaw/2)

    return [e0, e1, e2, e3]
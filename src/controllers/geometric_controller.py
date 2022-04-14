import sys

sys.path.append("C:/Users/Student/Documents/RLBot_IS/trajectory-optimization")

import numpy as np

from src.utilities.lin_alg_utils import LinAlgUtils as lau


class GeometricController:
    def __init__(self, threshold: float = 5):
        self.active = True
        self.threshold = 5.0

    def step(self, v: np.array, v_des: np.array, quat: np.array) -> list:
        """
        Returns the desired orientation vector based off of desired velocity

        Args:
            v (np.array): bot's current velocity vector
            v_des (np.array): desired velocity vector
            quat (np.array): current orientation of bot

        Returns:
            quat_des (np.array): desired orientation in quaternion form
        """
        v_error = v_des - v
        if not (
            np.absolute(v_error[0]) <= self.threshold
            and np.absolute(v_error[1]) <= self.threshold
            and np.absolute(v_error[2]) <= self.threshold
        ):
            quat_des = lau.quat_rotation_between_two_vectors(
                vect_0=np.array([1, 0, 0]), vect_1=v_error
            )
            quat_des = lau.quat_normalize(quat=quat_des)
        else:
            quat_des = quat
        return quat_des

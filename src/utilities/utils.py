import numpy as np


def convert_data(data: str) -> list:
    """
    Converts string data received by server to numpy arrays

    Args:
        data (str): string data with square brackets containing client gamestate information

    Returns:
        [q_bot, quat_des, controller_coefficients, g] (list): bot state, desired quaternion, coefficients for tuning, gravity
    """
    end_locations = [
        pos for pos, char in enumerate(data) if char == "]"
    ]  # look for ']' character to determine the end location of stringified list
    q_bot = np.fromstring(data[: end_locations[0] + 1].strip("[]"), count=13, sep=" ")
    quat_des = np.fromstring(
        data[end_locations[0] + 1 : end_locations[1] + 1].strip("[]"),
        count=4,
        sep=" ",
    )
    controller_coefficients = np.fromstring(
        data[end_locations[1] + 1 : end_locations[2] + 1].strip("[]"),
        count=2,
        sep=" ",
    )
    g = float(data[end_locations[2] + 1 :])
    return [q_bot, quat_des, controller_coefficients, g]

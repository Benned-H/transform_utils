"""Define utility functions for computations related to angles."""

import numpy as np


def wrap_angle_rad(angle_rad: float) -> float:
    """Wrap the given angle (in radians) into the interval [-pi, pi].

    :param angle_rad: Angle in radians
    :return: Equivalent angle, normalized into the interval [-pi, pi] radians
    """
    while angle_rad < -np.pi:
        angle_rad += 2 * np.pi
    while angle_rad > np.pi:
        angle_rad -= 2 * np.pi
    return angle_rad

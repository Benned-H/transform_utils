"""Define functions to support various common sampling operations."""

from __future__ import annotations

import numpy as np


def generate_alternating_centered_angles(center_rad: float, num_angles: int = 8) -> list[float]:
    """Generate angle samples that alternate symmetrically from a central target angle.

    The function produces a total of `num_angles` samples. It starts with the center angle
    (in radians) and alternates by stepping positively and negatively away from the center.
    The step size is determined by dividing the full circle [0, 2*pi] by (num_angles + 1).

    :param center_rad: Central angle (in radians) around which to generate samples
    :param num_angles: Total number of angle (in radians) samples to generate
    :return: List of evenly spaced sampled angles
    """
    assert num_angles > 0, f"Cannot sample {num_angles} angles."

    step_rad = 2 * np.pi / (num_angles + 1)  # Step (radians) between samples
    offset = 1  # Steps from center angle
    angles = [center_rad]

    while len(angles) < num_angles:
        angles.append(center_rad + offset * step_rad)
        if len(angles) < num_angles:
            angles.append(center_rad - offset * step_rad)
        offset += 1

    return angles[:num_angles]

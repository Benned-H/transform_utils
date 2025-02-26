"""Define functions to support various common sampling operations."""

from __future__ import annotations

import numpy as np


def sample_biased_angles(center_rad: float, num_samples: int = 8) -> list[float]:
    """Generate angle samples centered around a target angle, alternating outward on either side.

    Note: The sampled angles will be evenly spaced throughout the interval [0, 2*pi].

    :param center_rad: Center angle (radians) around which earlier samples are biased
    :param num_samples: Total number of angle samples (in radians) to generate
    :return: List of evenly spaced sampled angles
    """
    assert num_samples > 0, f"Cannot sample {num_samples} angles."

    step_rad = 2 * np.pi / (num_samples + 1)  # Step (radians) between samples
    steps_from_center = 1

    samples = [center_rad]
    while len(samples) < num_samples:
        greater_rad = center_rad + steps_from_center * step_rad
        lesser_rad = center_rad - steps_from_center * step_rad

        samples.append(greater_rad)
        samples.append(lesser_rad)

        steps_from_center += 1

    if len(samples) > num_samples:
        samples.pop()

    assert len(samples) == num_samples, f"Expected {num_samples} angles; found {len(samples)}"
    return samples

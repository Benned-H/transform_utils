"""Utility functions to compute weighted averages of 3D geometric primitives."""

from __future__ import annotations

from typing import Sequence

import numpy as np

from transform_utils.kinematics import Point3D, Pose3D, Quaternion


def compute_average_position(
    positions: Sequence[Point3D],
    weights: Sequence[float] | None = None,
) -> Point3D:
    """Compute a weighted average of 3D positions.

    :param positions: Sequence of Point3D objects (non-empty)
    :param weights: Optional sequence of per-pose weights; defaults to uniform weighting
    :return: A Point3D representing the weighted average
    :raises ValueError: If `positions` is empty or `weights` length mismatches
    """
    if not positions:
        raise ValueError("Cannot compute average of zero positions.")

    if weights is None:
        weights = [1.0] * len(positions)
    if len(weights) != len(positions):
        raise ValueError("Weights must have the same length as positions.")

    arr = np.vstack([p.to_array() for p in positions])  # shape (n, 3)
    avg = np.average(arr, axis=0, weights=weights)
    return Point3D.from_array(avg)


def compute_average_quaternion(
    quaternions: Sequence[Quaternion],
    weights: Sequence[float] | None = None,
) -> Quaternion:
    """Compute a maximum-likelihood average of quaternions.

    Uses eigen-decomposition of the weighted sum of outer products.
        Reference: Method 2 from this answer: https://math.stackexchange.com/a/3435296/614782

    :param quaternions: Sequence of Quaternion objects (non-empty)
    :param weights: Optional sequence of per-quaternion weights; defaults to uniform weighting
    :return: A Quaternion corresponding to the principal eigenvector
    :raises ValueError: If `quaternions` is empty or `weights` length mismatches
    """
    if not quaternions:
        raise ValueError("Cannot compute average of zero quaternions.")

    n = len(quaternions)
    if weights is None:
        weights = [1.0] * n
    if len(weights) != n:
        raise ValueError("Weights must have the same length as quaternions.")

    # Accumulate weighted outer products
    matrix = np.zeros((4, 4))
    for q, w in zip(quaternions, weights):
        v = q.to_array().reshape(4, 1)
        matrix += w * (v @ v.T)  # Sum of weighted outer products

    # Compute the principal eigenvector of the symmetric matrix
    _, vecs = np.linalg.eigh(matrix)
    principal = vecs[:, -1]
    return Quaternion.from_array(principal)


def compute_average_pose(
    poses: Sequence[Pose3D],
    weights: Sequence[float] | None = None,
) -> Pose3D:
    """Compute the spatial average of poses by averaging positions and orientations.

    :param poses: Sequence of Pose3D objects (non-empty)
    :param weights: Optional sequence of per-pose weights; defaults to uniform weighting
    :return: A Pose3D with averaged position and orientation
    :raises ValueError: If `poses` is empty or `weights` length mismatches
    """
    if not poses:
        raise ValueError("Cannot compute average of zero poses.")

    if weights is None:
        weights = [1.0] * len(poses)
    if len(weights) != len(poses):
        raise ValueError("Weights must have the same length as poses.")

    positions = [p.position for p in poses]
    orientations = [p.orientation for p in poses]

    avg_position = compute_average_position(positions, weights)
    avg_orientation = compute_average_quaternion(orientations, weights)
    return Pose3D(avg_position, avg_orientation)

"""Implement utility functions to compute averages over 3D geometric quantities."""

from __future__ import annotations

import numpy as np

from transform_utils.kinematics import Point3D, Pose3D, Quaternion


def compute_average_position(positions: list[Point3D], weights: list[float] | None) -> Point3D:
    """Compute the weighted average of the given 3D positions.

    :param poses: (x,y,z) positions in 3D space
    :param weights: Weights used for each position (equally weighted if None)
    :return: Average over the weighted positions
    """
    assert positions, "Cannot average a collection of zero positions."
    assert weights is None or len(positions) == len(weights), "Arguments differ in length."

    if weights is None:
        weights = [1] * len(positions)

    positions_arr = np.array([position.to_array() for position in positions])  # N x 3
    avg_position = np.average(positions_arr, axis=0, weights=weights)

    return Point3D.from_array(avg_position)


def compute_average_quaternion(
    quaternions: list[Quaternion],
    weights: list[float] | None,
) -> Quaternion:
    """Compute the maximum-likelihood average of the given quaternions.

    Reference: Method 2 from this answer: https://math.stackexchange.com/a/3435296/614782

    :param quaternions: Collection of quaternions to average
    :param weights: Weights used for each quaternion (equally weighted if None)
    :return: Maximum-likelihood average of the quaternions
    """
    assert quaternions, "Cannot average a collection of zero quaternions."
    assert weights is None or len(quaternions) == len(weights), "Arguments differ in length."

    if weights is None:
        weights = [1] * len(quaternions)

    accum = np.zeros((4, 4))
    for quat, weight in zip(quaternions, weights):
        q_vec = quat.to_array()
        weighted_outer_product = weight * np.outer(q_vec, q_vec)
        accum += weighted_outer_product

    _, eigenvectors = np.linalg.eig(accum)
    ev0 = eigenvectors[:, 0]

    return Quaternion.from_array(ev0)


def compute_average_pose(poses: list[Pose3D], weights: list[float] | None) -> Pose3D:
    """Compute the 'spatial average' of the given 3D poses.

    :param poses: Positions and orientations in 3D space
    :param weights: Weights used for each quaternion (equally weighted if None)
    :return: Average over all weighted positions and orientations
    """
    assert poses, "Cannot average a collection of zero poses."
    assert weights is None or len(poses) == len(weights), "Arguments differ in length."

    if weights is None:
        weights = [1] * len(poses)

    avg_position = compute_average_position([p.position for p in poses], weights)
    avg_orientation = compute_average_quaternion([p.orientation for p in poses], weights)

    return Pose3D(avg_position, avg_orientation)

"""Define utility functions used to calculate metric distances."""

from __future__ import annotations

import numpy as np

from transform_utils.kinematics import Pose3D


def compute_distances_to_others_m(pose_idx: int, poses: list[Pose3D]) -> list[float]:
    """Compute the Euclidean distance (m) from the index-specified pose to all others.

    :param pose_idx: Index of the pose from which distances are calculated
    :param poses: Collection of poses used in distance calculations
    :return: List of the distances (meters) from the index-specified pose to each other pose
    """
    assert len(poses) > 1, "Cannot compute distances using only one pose."
    assert 0 <= pose_idx < len(poses), f"Invalid index {pose_idx} for list of length {len(poses)}."

    origin_arr = poses[pose_idx].position.to_array()
    other_positions = [p_i.position.to_array() for i, p_i in enumerate(poses) if i != pose_idx]

    return [np.linalg.norm(origin_arr - other_arr) for other_arr in other_positions]


def identify_worst_outlier(poses: list[Pose3D]) -> int:
    """Find the index of the pose in the given list that's farthest from the others, on average.

    :param poses: Collection of 3D poses
    :return: Index of the "worst" outlier pose, by average Euclidean distance (m) to other poses
    """
    assert poses, "Cannot identify an outlier from zero poses."

    worst_idx = None
    worst_avg_distance_m = -1
    for idx, _ in enumerate(poses):
        avg_distance_m = np.mean(compute_distances_to_others_m(idx, poses))

        if avg_distance_m > worst_avg_distance_m:
            worst_idx = idx
            worst_avg_distance_m = avg_distance_m

    return worst_idx

"""Define utility functions to compute various distance metrics."""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from transform_utils.kinematics import DEFAULT_FRAME, Pose2D, Pose3D
from transform_utils.math.angles import wrap_angle_rad
from transform_utils.transform_manager import TransformManager


def euclidean_distance_3d_m(pose_a: Pose2D | Pose3D, pose_b: Pose2D | Pose3D) -> float:
    """Compute the Euclidean distance (in meters) between two poses in 3D space.

    Note: This calculation ignores the orientations of the poses.

    :param pose_a: First pose (2D or 3D) used to compute the 3D distance
    :param pose_b: Second pose (2D or 3D) used to compute the 3D distance
    :return: Straight-line distance (meters) between the two poses in 3D space
    """
    if pose_a.ref_frame != pose_b.ref_frame:
        pose_a = TransformManager.convert_to_frame(pose_a, DEFAULT_FRAME)
        pose_b = TransformManager.convert_to_frame(pose_b, DEFAULT_FRAME)

    if isinstance(pose_a, Pose2D):
        pose_a = Pose3D.from_2d(pose_a)
    if isinstance(pose_b, Pose2D):
        pose_b = Pose3D.from_2d(pose_b)

    return np.linalg.norm(pose_a.position.to_array(), pose_b.position.to_array())


def euclidean_distance_2d_m(pose_a: Pose2D | Pose3D, pose_b: Pose2D | Pose3D) -> float:
    """Compute the Euclidean distance (in meters) between two poses in 2D space.

    Note: This calculation ignores the orientations and z-values of the poses.

    :param pose_a: First pose (2D or 3D) used to compute the 2D distance
    :param pose_b: Second pose (2D or 3D) used to compute the 2D distance
    :return: Straight-line distance (meters) between the two poses in 2D space
    """
    if pose_a.ref_frame != pose_b.ref_frame:
        pose_a = TransformManager.convert_to_frame(pose_a, DEFAULT_FRAME)
        pose_b = TransformManager.convert_to_frame(pose_b, DEFAULT_FRAME)

    if isinstance(pose_a, Pose3D):
        pose_a = pose_a.to_2d()
    if isinstance(pose_b, Pose3D):
        pose_b = pose_b.to_2d()

    return np.linalg.norm(np.array([pose_a.x - pose_b.x, pose_a.y - pose_b.y]))


def absolute_yaw_error_rad(pose_a: Pose2D | Pose3D, pose_b: Pose2D | Pose3D) -> float:
    """Compute the absolute yaw error (in radians) between two poses.

    :param pose_a: First pose (2D or 3D) used to compute the yaw error
    :param pose_b: Second pose (2D or 3D) used to compute the yaw error
    :return: Absolute yaw error (radians, between 0 and pi) between the poses
    """
    yaw_error_rad = wrap_angle_rad(pose_a.yaw_rad - pose_b.yaw_rad)
    return np.abs(yaw_error_rad)


def compute_distances_to_others_m(pose_idx: int, poses: Sequence[Pose3D]) -> list[float]:
    """Compute Euclidean distances (in meters) from the specified pose to all others.

    :param pose_idx: Index of the reference pose in `poses`
    :param poses: Sequence of Pose3D objects (length > 1)
    :return: List of distances from `poses[pose_idx]` to each other pose
    :raises ValueError: If fewer than 2 poses are provided or `pose_idx` is out of range
    """
    if len(poses) < 2:
        error_msg = f"Need at least 2 poses to compute distances, got {len(poses)}."
        raise ValueError(error_msg)

    if not (0 <= pose_idx < len(poses)):
        error_msg = f"Index {pose_idx} out of range for poses sequence of length {len(poses)}."
        raise ValueError(error_msg)

    origin = poses[pose_idx].position.to_array()
    distances = []
    for pose in poses:
        other = pose.position.to_array()
        distances.append(float(np.linalg.norm(origin - other)))
    return distances


def identify_worst_outlier(poses: Sequence[Pose3D]) -> int:
    """Identify the index of the pose farthest (on average) from the others.

    :param poses: Sequence of Pose3D objects (non-empty)
    :return: Index of the pose with the largest mean distance to other poses
    :raises ValueError: If `poses` is empty
    """
    if not poses:
        error_msg = "Cannot identify an outlier from zero poses."
        raise ValueError(error_msg)

    worst_idx = 0
    worst_avg_dist_m = -1.0
    for idx in range(len(poses)):
        distances = compute_distances_to_others_m(idx, poses)
        avg_dist_m = float(np.mean(distances))
        if avg_dist_m > worst_avg_dist_m:
            worst_avg_dist_m = avg_dist_m
            worst_idx = idx

    return worst_idx

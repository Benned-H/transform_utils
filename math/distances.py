"""Define utility functions to compute various distance metrics."""

from __future__ import annotations

import numpy as np

from transform_utils.kinematics import Pose2D, Pose3D
from transform_utils.math.angles import wrap_angle_rad


def euclidean_distance_m(pose_a: Pose2D | Pose3D, pose_b: Pose2D | Pose3D) -> float:
    """Compute the Euclidean distance (in meters) between two poses.

    Note: This calculation ignores the orientations of the given poses.

    :param pose_a: First pose (2D or 3D) used in the distance computation
    :param pose_b: Second pose (2D or 3D) used in the distance computation
    :return: Straight-line distance (meters) between the two poses
    """
    if isinstance(pose_a, Pose2D):
        pose_a = Pose3D.from_2d(pose_a)
    if isinstance(pose_b, Pose2D):
        pose_b = Pose3D.from_2d(pose_b)

    return np.linalg.norm(pose_a.position.to_array(), pose_b.position.to_array())


def absolute_yaw_error_rad(pose_a: Pose2D | Pose3D, pose_b: Pose2D | Pose3D) -> float:
    """Compute the absolute yaw error (in radians) between two poses.

    :param pose_a: First pose (2D or 3D) used to compute the yaw error
    :param pose_b: Second pose (2D or 3D) used to compute the yaw error
    :return: Absolute yaw error (radians, between 0 and pi) between the poses
    """
    yaw_error_rad = wrap_angle_rad(pose_a.yaw_rad - pose_b.yaw_rad)
    return np.abs(yaw_error_rad)

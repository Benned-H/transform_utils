"""Define utility functions to compute various distance metrics."""

from __future__ import annotations

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

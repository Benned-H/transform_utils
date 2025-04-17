"""Define functions for loading object and environment models from YAML (without needing ROS)."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import yaml

from transform_utils.kinematics import Pose2D, Pose3D
from transform_utils.logging import log_error, log_info

if TYPE_CHECKING:
    from pathlib import Path


def load_yaml_into_dict(yaml_path: Path) -> dict[str, Any]:
    """Load data from a YAML file into a Python dictionary.

    :param yaml_path: Path to the YAML file to be imported
    :return: Dictionary mapping strings to values (empty if the YAML file is nonexistent/invalid)
    """
    if not yaml_path.exists():
        log_error(f"The YAML path {yaml_path} doesn't exist!")
        return {}

    try:
        with yaml_path.open() as yaml_file:
            yaml_data = yaml.safe_load(yaml_file)
            log_info(f"Loaded data from YAML file: {yaml_path}")

    except yaml.YAMLError as error:
        log_error(f"Failed to load YAML file: {yaml_path}\nError: {error}")
        return {}

    return yaml_data


def load_named_poses_2d(landmarks_data: dict[str, Any], default_frame: str) -> dict[str, Pose2D]:
    """Load a collection of named 2D poses from data imported from YAML.

    :param landmarks_data: Map from location names to the corresponding 2D pose data
    :param default_frame: Reference frame used for any poses without a specified frame
    :return: Map from robot/object/location names to 2D poses
    """
    named_poses: dict[str, Pose2D] = {}

    for name, pose_data in landmarks_data.items():
        if isinstance(pose_data, list):  # Pose represented as [x, y, yaw] list
            named_poses[name] = Pose2D.from_list(pose_data, ref_frame=default_frame)
        elif isinstance(pose_data, dict):  # Pose with reference frame specified
            ref_frame: str = pose_data.get("frame", default_frame)
            pose_list = pose_data["x_y_yaw"]
            named_poses[name] = Pose2D.from_list(pose_list, ref_frame=ref_frame)

    return named_poses


def load_named_poses(poses_data: dict[str, Any], default_frame: str) -> dict[str, Pose3D]:
    """Load a set of named 3D poses from data imported from YAML.

    :param poses_data: Dictionary mapping robot/object/location names to 3D pose data
    :param default_frame: Reference frame used for any poses without a specified frame
    :return: Map from robot/object/location names to 3D poses
    """
    named_poses: dict[str, Pose3D] = {}

    for name, pose_data in poses_data.items():
        if isinstance(pose_data, list):  # Pose represented as XYZ-RPY list
            named_poses[name] = Pose3D.from_list(pose_data, ref_frame=default_frame)
        elif isinstance(pose_data, dict):  # Pose with reference frame specified
            ref_frame: str = pose_data.get("frame", default_frame)
            pose_list = pose_data["xyz_rpy"]
            named_poses[name] = Pose3D.from_list(pose_list, ref_frame=ref_frame)

    return named_poses

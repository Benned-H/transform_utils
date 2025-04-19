"""Define functions for loading object and environment models from YAML (without needing ROS)."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import yaml

from transform_utils.kinematics import DEFAULT_FRAME, Pose2D, Pose3D
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
        named_poses[name] = Pose3D.from_yaml(pose_data, default_frame=default_frame)

    return named_poses


def load_known_landmarks(yaml_path: Path) -> dict[str, Pose2D]:
    """Load landmark poses from the given YAML file.

    :param yaml_path: Path to a YAML file containing landmark pose data
    :return: Dictionary mapping landmark names to their imported 2D poses
    """
    yaml_data = load_yaml_into_dict(yaml_path)
    default_frame = yaml_data.get("default_frame", DEFAULT_FRAME)
    landmarks_data = yaml_data.get("known_landmarks", {})

    if not landmarks_data:
        log_error(f"Expected to find the key 'known_landmarks' in YAML file: {yaml_path}")
        return {}

    return {
        landmark_name: Pose2D.from_yaml(pose_data, default_frame)
        for (landmark_name, pose_data) in landmarks_data.items()
    }


def load_object_poses(yaml_path: Path) -> dict[str, Pose3D]:
    """Load known object poses from the given YAML file.

    :param yaml_path: Path to a YAML file containing object pose data
    :return: Dictionary mapping object names to their imported 3D poses
    """
    yaml_data = load_yaml_into_dict(yaml_path)
    default_frame = yaml_data.get("default_frame", DEFAULT_FRAME)
    object_poses_data = yaml_data.get("object_poses", {})

    if not object_poses_data:
        log_error(f"Expected to find the key 'object_poses_data' in YAML file: {yaml_path}")
        return {}

    return {
        obj_name: Pose3D.from_yaml(pose_data, default_frame)
        for (obj_name, pose_data) in object_poses_data.items()
    }


def load_robot_base_poses(yaml_path: Path) -> dict[str, Pose3D]:
    """Load robot base poses from the given YAML file.

    :param yaml_path: Path to a YAML file containing robot base pose data
    :return: Dictionary mapping robot names to their imported 3D poses
    """
    yaml_data = load_yaml_into_dict(yaml_path)
    default_frame = yaml_data.get("default_frame", DEFAULT_FRAME)
    base_poses_data = yaml_data.get("robot_base_poses", {})

    if not base_poses_data:
        log_error(f"Expected to find the key 'robot_base_poses' in YAML file: {yaml_path}")
        return {}

    return {
        robot_name: Pose3D.from_yaml(pose_data, default_frame)
        for (robot_name, pose_data) in base_poses_data.items()
    }

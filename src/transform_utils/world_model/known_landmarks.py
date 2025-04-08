"""Define a dataclass representing a set of named 2D landmarks."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from transform_utils.filesystem.load_from_yaml import load_named_poses_2d, load_yaml_into_dict
from transform_utils.kinematics import DEFAULT_FRAME, Pose2D

if TYPE_CHECKING:
    from pathlib import Path


@dataclass
class KnownLandmarks2D:
    """A collection of named landmarks at known 2D poses."""

    landmarks: dict[str, Pose2D]

    @classmethod
    def from_yaml(cls, yaml_path: Path) -> KnownLandmarks2D:
        """Construct a KnownLandmarks2D instance from a YAML file.

        Note: The YAML file is expected to contain a "known_landmarks" key whose value is a list
            of landmark specifications (i.e., name mapped to a list representing a 2D pose), and
            an optional "default_frame" key whose value is the default frame used for the 2D poses.

        :param yaml_path: Path to a YAML file containing landmarks data
        :return: Constructed KnownLandmarks2D instance
        """
        yaml_data: dict[str, Any] = load_yaml_into_dict(yaml_path)

        # Extract the 'known_landmarks' and 'default_frame' values
        landmarks_data = yaml_data.get("known_landmarks", {})
        default_frame = yaml_data.get("default_frame", DEFAULT_FRAME)

        landmarks = load_named_poses_2d(landmarks_data, default_frame)

        return cls(landmarks=landmarks)

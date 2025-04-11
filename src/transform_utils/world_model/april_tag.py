"""Define a dataclass to represent an AprilTag (i.e., an AR marker or visual fiducial)."""

from __future__ import annotations

from dataclasses import dataclass

from transform_utils.kinematics import Pose3D


@dataclass
class AprilTag:
    """An AR marker at some pose in the world."""

    id: int  # Unique ID of the tag
    size_cm: float  # Size (in centimeters) of one side of the tag's black square
    pose: Pose3D | None  # Estimated pose of the AprilTag (None if unknown)

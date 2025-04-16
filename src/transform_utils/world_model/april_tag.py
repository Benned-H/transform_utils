"""Define a dataclass to represent an AprilTag (i.e., an AR marker or visual fiducial)."""

from __future__ import annotations

from dataclasses import dataclass
from math import isclose
from pathlib import Path
from typing import Any

from transform_utils.filesystem.load_from_yaml import load_yaml_into_dict
from transform_utils.kinematics import Pose3D


@dataclass
class AprilTag:
    """An AR marker at some pose in the world."""

    id: int  # Unique ID of the tag
    size_cm: float  # Size (in centimeters) of one side of the tag's black square
    pose: Pose3D | None  # Estimated pose of the AprilTag (None if unknown)
    relative_frames: dict[str, Pose3D]  # Object frames w.r.t. the AprilTag

    @classmethod
    def from_yaml(cls, tag_name: str, tag_data: dict[str, Any]) -> AprilTag:
        """Construct an AprilTag instance from imported YAML data.

        :param tag_name: Name of the tag (of the form "tag_XX")
        :param tag_data: AprilTag data imported from YAML
        :return: Constructed AprilTag instance
        """
        assert tag_name.startswith("tag_"), f"{tag_name} is not of the form 'tag_XX'."
        assert "tag_size_cm" in tag_data, f"Expected tag {tag_name} to have key 'tag_size_cm'."

        tag_id = int(tag_name[4:])
        tag_size_cm = tag_data["tag_size_cm"]
        output_tag = AprilTag(id=tag_id, size_cm=tag_size_cm, pose=None, relative_frames={})

        frames = tag_data.get("relative_frames", {})  # Map from frame names to relative poses
        for child_frame_name, pose_data in frames.items():
            relative_pose = Pose3D.from_yaml(pose_data, default_frame=tag_name)
            output_tag.relative_frames[child_frame_name] = relative_pose

        return output_tag

    @property
    def frame_name(self) -> str:
        """Retrieve the name of the /tf frame defined by this AprilTag."""
        return f"tag_{self.id}"


@dataclass
class AprilTagSystem:
    """A system of known AprilTags and tag-detecting cameras."""

    tags: dict[int, AprilTag]  # Map tag IDs to AprilTag instances
    camera_detects_tags: dict[str, set[int]]  # Map camera names to the tags they can detect
    camera_topic_prefix: str  # Prefix for AR marker detection ROS topics

    @classmethod
    def from_yaml(cls, yaml_path: Path) -> AprilTagSystem:
        """Load a system of AprilTags and tag-detecting cameras from a YAML file.

        :param yaml_path: Path to a YAML file specifying AprilTag and camera data
        :return: Constructed AprilTagSystem instance
        """
        yaml_data = load_yaml_into_dict(yaml_path)
        assert "tags" in yaml_data, f"Expected key 'tags' in YAML file {yaml_path}."

        topic_prefix = yaml_data.get("camera_topic_prefix", "")
        system = AprilTagSystem(tags={}, camera_detects_tags={}, camera_topic_prefix=topic_prefix)

        for tag_name, tag_data in yaml_data["tags"].items():
            new_tag = AprilTag.from_yaml(tag_name, tag_data)
            system.tags[new_tag.id] = new_tag

        assert "cameras" in yaml_data, f"Expected key 'cameras' in YAML file {yaml_path}."
        for camera_name, camera_data in yaml_data["cameras"].items():
            system.camera_detects_tags[camera_name] = set()

            # Check if each tag's size is approximately one that this camera recognizes
            recognized_sizes_cm: set[float] = set(camera_data["recognized_tag_sizes_cm"])
            for tag in system.tags.values():
                for recognized_size_cm in recognized_sizes_cm:
                    if isclose(tag.size_cm, recognized_size_cm):
                        system.camera_detects_tags[camera_name].add(tag.id)
                        break  # Move to the next AprilTag

        return system

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
    in_bundle: bool  # Specifies whether the tag is part of a larger bundle

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
        in_bundle = tag_data.get("bundle", False)
        output_tag = AprilTag(
            id=tag_id,
            size_cm=tag_size_cm,
            pose=None,
            relative_frames={},
            in_bundle=in_bundle,
        )

        frames = tag_data.get("relative_frames", {})  # Map from frame names to relative poses
        for child_frame_name, pose_data in frames.items():
            relative_pose = Pose3D.from_yaml(pose_data, default_frame=tag_name)
            relative_pose.frame_id = tag_name
            output_tag.relative_frames[child_frame_name] = relative_pose

        return output_tag

    @property
    def frame_name(self) -> str:
        """Retrieve the name of the /tf frame defined by this AprilTag."""
        return f"tag_{self.id}"


@dataclass
class TagDetectingCamera:
    """A camera that detects AR markers."""

    recognized_tag_sizes_cm: set[float]  # Sizes of AR markers detected by the camera
    detects_single_tags: bool  # Whether this camera is configured to detect single tags
    detects_bundles: bool  # Whether this camera is configured to detect bundles of tags

    @classmethod
    def from_yaml(cls, camera_name: str, camera_data: dict[str, Any]) -> TagDetectingCamera:
        """Construct a TagDetectingCamera instance from imported YAML data.

        :param camera_name: Name of the camera (e.g., "hand")
        :param camera_data: Camera data imported from YAML
        :return: Constructed TagDetectingCamera instance
        """
        required_keys = ["recognized_tag_sizes_cm", "detects_single_tags", "detects_bundles"]
        for key in required_keys:
            assert key in camera_data, f"Expected key '{key}' in data for camera '{camera_name}'."

        recognized_sizes_cm: set[float] = set(camera_data["recognized_tag_sizes_cm"])
        detects_single = bool(camera_data["detects_single_tags"])
        detects_bundle = bool(camera_data["detects_bundles"])

        return TagDetectingCamera(
            recognized_tag_sizes_cm=recognized_sizes_cm,
            detects_single_tags=detects_single,
            detects_bundles=detects_bundle,
        )

    def can_recognize_tag(self, tag: AprilTag) -> bool:
        """Evaluate whether the camera, as configured, can recognize the given tag.

        :param tag: An AR marker with some size (cm), possibly in a tag bundle
        :return: True if the camera can recognize the tag, else False
        """
        size_ok = any(isclose(tag.size_cm, size_cm) for size_cm in self.recognized_tag_sizes_cm)
        bundle_ok = self.detects_bundles or (not tag.in_bundle)
        single_ok = self.detects_single_tags or tag.in_bundle

        return size_ok and bundle_ok and single_ok


@dataclass
class AprilTagSystem:
    """A system of known AprilTags and tag-detecting cameras."""

    tags: dict[int, AprilTag]  # Map tag IDs to AprilTag instances
    cameras: dict[str, TagDetectingCamera]  # Map camera names to TagDetectingCamera instances
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
        system = AprilTagSystem({}, {}, {}, topic_prefix)

        for tag_name, tag_data in yaml_data["tags"].items():
            new_tag = AprilTag.from_yaml(tag_name, tag_data)
            system.tags[new_tag.id] = new_tag

        assert "cameras" in yaml_data, f"Expected key 'cameras' in YAML file {yaml_path}."
        for camera_name, camera_data in yaml_data["cameras"].items():
            camera = TagDetectingCamera.from_yaml(camera_name, camera_data)
            system.cameras[camera_name] = camera
            system.camera_detects_tags[camera_name] = set()

            for tag in system.tags.values():
                if camera.can_recognize_tag(tag):
                    system.camera_detects_tags[camera_name].add(tag.id)

        return system

"""Define a dataclass to represent a physical object in the environment."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from transform_utils.kinematics import DEFAULT_FRAME, Pose3D


@dataclass
class Object:
    """A typed object at some pose in the environment."""

    name: str
    object_type: str
    pose: Pose3D | None  # Pose of the object (None if the pose is unknown)
    is_static: bool  # True indicates that the object pose does not change

    # TODO: Load subframes if desired

    @property
    def type(self) -> str:
        """Get the type of the object."""
        return self.object_type

    @classmethod
    def from_yaml(cls, object_name: str, object_data: dict[str, Any]) -> Object:
        """Construct an Object instance from a dictionary of YAML data.

        :param object_name: Name of the object
        :param object_data: Dictionary of object data imported from YAML
        :return: Constructed Object instance
        """
        assert "type" in object_data, f"Expected 'type' key in data for object '{object_name}'."

        obj_type = object_data["type"]
        is_static = object_data.get("static", False)

        pose_data = object_data.get("pose")
        pose = None if pose_data is None else Pose3D.from_yaml(pose_data, DEFAULT_FRAME)

        return Object(name=object_name, object_type=obj_type, pose=pose, is_static=is_static)

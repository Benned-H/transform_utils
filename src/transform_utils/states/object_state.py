"""Model the kinematic state of an object, including its current pose and collision model."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from transform_utils.kinematics import DEFAULT_FRAME, Pose3D
from transform_utils.states.collision_models import CollisionModel


@dataclass
class ObjectState:
    """A kinematic state of a typed, rigid-body object."""

    name: str  # Name of the object (identifies its frame)
    object_type: str  # Type of the object
    pose: Pose3D  # Pose of the object (specifies a relative parent frame)
    collision_model: CollisionModel  # Rigid-body model of the object

    @classmethod
    def from_yaml(cls, object_name: str, object_data: dict[str, Any]) -> ObjectState:
        """Construct an ObjectState instance from a dictionary of YAML data.

        :param object_name: Name of the object to be imported
        :param object_data: Dictionary of object data imported from YAML
        :return: Constructed ObjectState instance
        """
        assert "type" in object_data, f"Expected key 'type' in data for object '{object_name}'."
        assert "mesh" in object_data or "geometry" in object_data

        object_type = object_data["type"]

        pose_data = object_data.get("pose")
        pose = None if pose_data is None else Pose3D.from_yaml(pose_data, DEFAULT_FRAME)

        collision_model = CollisionModel.from_yaml(object_data)

        return ObjectState(object_name, object_type, pose, collision_model)

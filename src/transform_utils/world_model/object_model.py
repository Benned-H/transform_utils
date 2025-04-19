"""Define a dataclass to represent an object in the environment."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from transform_utils.world_model.collision_models import CollisionModel


@dataclass
class ObjectModel:
    """A typed object with collision geometry."""

    name: str
    object_type: str
    collision_model: CollisionModel

    @classmethod
    def from_yaml(cls, object_name: str, object_data: dict[str, Any]) -> ObjectModel:
        """Construct an ObjectModel instance from a dictionary of YAML data.

        :param object_name: Name of the object to be imported
        :param object_data: Dictionary of object data imported from YAML
        :return: Constructed ObjectModel instance
        """
        assert "type" in object_data, f"Expected key 'type' in data for object '{object_name}'."
        assert "mesh" in object_data or "geometry" in object_data

        obj_type = object_data["type"]
        collision_model = CollisionModel.from_yaml(object_data)

        return ObjectModel(object_name, obj_type, collision_model)

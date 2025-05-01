"""Define a class to represent an object-centric world with collision geometry."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from transform_utils.filesystem.load_from_yaml import load_yaml_into_dict
from transform_utils.world_model.kinematic_state import KinematicState
from transform_utils.world_model.known_landmarks import KnownLandmarks2D
from transform_utils.world_model.object_model import ObjectModel


@dataclass
class WorldModel:
    """An object-centric world with collision geometry."""

    kinematic_state: KinematicState  # State of all spatial information in the environment
    object_models: dict[str, ObjectModel]  # Maps object names to their models
    landmarks: KnownLandmarks2D  # Named landmarks at known 2D poses

    @classmethod
    def from_yaml(cls, yaml_path: Path) -> WorldModel:
        """Construct a WorldModel instance using data from the given YAML file.

        :param yaml_path: Path to a YAML file containing world data
        :return: Constructed WorldModel instance
        """
        yaml_data = load_yaml_into_dict(yaml_path)
        objects = {}
        if "objects" in yaml_data:
            for object_name, object_data in yaml_data["objects"].items():
                objects[object_name] = ObjectModel.from_yaml(object_name, object_data)

        return WorldModel(
            kinematic_state=KinematicState.from_yaml(yaml_path),
            object_models=objects,
            landmarks=KnownLandmarks2D.from_yaml(yaml_path),
        )

    @property
    def robot_names(self) -> list[str]:
        """Retrieve the names of all robots in the world model."""
        return list(self.kinematic_state.robot_base_poses.keys())

    @property
    def object_names(self) -> list[str]:
        """Retrieve the names of all objects in the world model."""
        objects_with_models = set(self.object_models.keys())
        objects_with_poses = set(self.kinematic_state.object_poses.keys())

        assert objects_with_models == objects_with_poses
        return objects_with_poses

"""Define a dataclass to represent an object in the environment."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from moveit_msgs.msg import CollisionObject

    from transform_utils.kinematics import Pose3D


@dataclass
class ObjectModel:
    """An object with collision geometry at some pose in the environment."""

    message: CollisionObject
    pose: Pose3D | None  # Pose of the object, or None if its pose is unknown
    dimensions: tuple[float, float, float]  # Size in (x, y, z)
    static: bool  # True if the object cannot move

    @property
    def name(self) -> str:
        """Get the name of the object."""
        return str(self.message.id)

    @property
    def type(self) -> str:
        """Get the type of the object."""
        return str(self.message.type.key)

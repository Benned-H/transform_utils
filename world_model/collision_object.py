"""Define a dataclass to represent a collision object."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from moveit_msgs.msg import CollisionObject as CollisionObjectMsg

    from transform_utils.kinematics import Pose3D


@dataclass
class CollisionObject:
    """A geometric collision model of an object."""

    message: CollisionObjectMsg
    pose: Pose3D | None  # Pose of the object w.r.t. its relative frame (None = pose unknown)
    dimensions: tuple[float, float, float]  # Size in (x, y, z)

    @property
    def name(self) -> str:
        """Get the name of the collision object."""
        return self.message.id

    @property
    def type(self) -> str:
        """Get the object type of the collision object."""
        return self.message.type.key

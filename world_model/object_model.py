"""Define a dataclass to represent an object in the environment."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from moveit_msgs.msg import CollisionObject


@dataclass
class ObjectModel:
    """An object with collision geometry at some pose in the environment."""

    message: CollisionObject
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

    def get_yaml_data(self) -> dict:
        """Convert the ObjectModel to a dictionary of its YAML data."""
        if self.pose.estimated_pose is not None:
            # Convert pose to a list: [x, y, z, roll, pitch, yaw]
            xyz, rpy = self.pose.estimated_pose.to_xyz_rpy()
            pose_list = list(xyz) + list(rpy)

        return {
            "type": self.type,
            "pose": pose_list,
            "frame": self.pose.estimated_pose.ref_frame,
        }

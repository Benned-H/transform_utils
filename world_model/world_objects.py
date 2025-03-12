"""Define a dataclass to model the objects in the environment."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Generic, TypeVar

from transform_utils.world_model.pose_estimate import AbstractPoseEstimate

if TYPE_CHECKING:
    from transform_utils.kinematics import Pose3D
    from transform_utils.world_model.object_model import ObjectModel


PoseEstimateT = TypeVar("PoseEstimateT", bound=AbstractPoseEstimate)


@dataclass
class WorldObjects(Generic[PoseEstimateT]):
    """A collection of object models at estimated poses in the world."""

    objects: dict[str, ObjectModel]  # Map from object names to object models
    pose_estimates: dict[str, PoseEstimateT]  # Map from object names to pose estimates

    def get_object(self, obj_name: str) -> ObjectModel:
        """Retrieve the object of the given name."""
        assert obj_name in self.objects, f"Unknown object name: '{obj_name}'."
        return self.objects[obj_name]

    def get_pose(self, obj_name: str) -> Pose3D | None:
        """Retrieve the best available estimated pose of the named object."""
        assert obj_name in self.pose_estimates, f"No pose estimate is available for '{obj_name}'."
        return self.pose_estimates[obj_name].pose

    def set_pose(self, obj_name: str, new_pose: Pose3D) -> None:
        """Set the pose of the named object to the given pose.

        Note: The stored pose estimate is overwritten to just the given pose.

        :param obj_name: Name of the object assigned the given pose
        :param new_pose: New 3D pose of the object (directly assigned)
        """
        assert obj_name in self.objects, f"Cannot set pose of unknown object: '{obj_name}'."

        self.pose_estimates[obj_name].reset_to(new_pose)

    def update_pose_estimate(self, obj_name: str, pose: Pose3D, confidence: float = 0.0) -> None:
        """Update the pose estimate of the named object using the given data.

        :param obj_name: Name of the object whose pose estimate is updated
        :param pose: Newly available pose estimate
        :param confidence: Confidence value associated with the estimate, defaults to 0.0
        """
        assert obj_name in self.pose_estimates, f"Cannot update an unknown object: '{obj_name}'."

        self.pose_estimates[obj_name].update(pose, confidence)

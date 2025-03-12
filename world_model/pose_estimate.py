"""Define dataclasses to represent the estimated poses of objects."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING

from transform_utils.math.average_3d import compute_average_pose

if TYPE_CHECKING:
    from transform_utils.kinematics import Pose3D


class AbstractPoseEstimate(ABC):
    """An abstract class defining the necessary properties of any pose estimate class."""

    @property
    @abstractmethod
    def pose(self) -> Pose3D | None:
        """Retrieve the current pose estimate (or None if no estimate exists)."""

    @property
    @abstractmethod
    def known(self) -> bool:
        """Retrieve whether the stored pose estimate is considered 'known'."""

    @abstractmethod
    def reset_to(self, new_pose: Pose3D) -> None:
        """Reset the stored pose estimate to the given pose.

        :param new_pose: New stored pose after the pose estimate is reset
        """

    @abstractmethod
    def update(self, new_pose: Pose3D, confidence: float = 0.0) -> None:
        """Update the pose estimate using the given pose estimate.

        :param new_pose: New 3D pose to integrate into the stored estimate
        :param confidence: Confidence value associated with the given pose, defaults to 0.0
        """


@dataclass
class PoseEstimate3D:
    """An object pose estimate with an associated confidence value."""

    pose: Pose3D
    confidence: float = 0.0  # Confidence associated with this estimate


class AveragedPoseEstimate3D(AbstractPoseEstimate):
    """An object pose estimate based on an average of observed pose estimates."""

    def __init__(self) -> None:
        """Initialize the member variables of the averaged pose estimate."""
        self._estimates: set[PoseEstimate3D] = set()

    @property
    def pose(self) -> Pose3D | None:
        """Retrieve the current pose estimate (or None if no estimate exists).

        Note: The estimate of the AveragedPoseEstimate3D averages over the stored pose estimates.

        :return: Averaged pose estimate, or None if no data is available.
        """
        if not self._estimates:
            return None

        # For now, ignore each estimate's confidence. We could instead compute a weighted average.
        poses = [e.pose for e in self._estimates]
        return compute_average_pose(poses, weights=None)

    @property
    def known(self) -> bool:
        """Retrieve whether the averaged pose estimate is considered 'known'.

        :return: True if any stored estimate has confidence greater than zero, else False.
        """
        max_confidence = max([e.confidence for e in self._estimates], default=0.0)
        return max_confidence > 0.0

    def reset_to(self, new_pose: Pose3D, confidence: float = 100.0) -> None:
        """Reset the stored pose estimate to the given pose.

        :param new_pose: New stored pose after the pose estimate is reset
        :param confidence: Confidence value associated with the pose, defaults to 100.0
        """
        self._estimates.clear()
        self.update(new_pose, confidence)

    def update(self, new_pose: Pose3D, confidence: float = 0.0) -> None:
        """Update the stored pose estimate using the given pose estimate.

        :param new_pose: New 3D pose to integrate into the combined estimate
        :param confidence: Confidence value associated with the given pose, defaults to 0.0
        """
        self._estimates.add(PoseEstimate3D(new_pose, confidence))

    @property
    def all_estimates(self) -> set[PoseEstimate3D]:
        """Retrieve the set of all pose estimates forming the average."""
        return self._estimates

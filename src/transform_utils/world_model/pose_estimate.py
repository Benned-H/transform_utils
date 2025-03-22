"""Define dataclasses to represent the estimated poses of objects."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING

from transform_utils.math.average_3d import compute_average_pose
from transform_utils.math.distances import compute_distances_to_others_m, identify_worst_outlier

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
    def reset(self) -> None:
        """Reset the pose estimate to its initial empty state."""

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

    def __init__(self, max_estimates: int = 10) -> None:
        """Initialize the member variables of the averaged pose estimate.

        :param max_estimates: Maximum number of estimates to track at once
        """
        self._estimates: list[PoseEstimate3D] = []
        self.max_estimates = max_estimates

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

    def reset(self) -> None:
        """Reset the pose estimate to its initial empty state."""
        self._estimates.clear()

    def update(self, new_pose: Pose3D, confidence: float = 0.0) -> None:
        """Update the stored pose estimate using the given pose estimate.

        :param new_pose: New 3D pose to integrate into the combined estimate
        :param confidence: Confidence value associated with the given pose, defaults to 0.0
        """
        self._estimates.append(PoseEstimate3D(new_pose, confidence))

        # Prune estimates if we've exceeded the maximum intended number
        while len(self._estimates) > self.max_estimates:
            all_poses = [est.pose for est in self._estimates]

            # Filter any estimate more than a meter from all other estimates
            for idx in range(len(self._estimates)):
                distances_m = compute_distances_to_others_m(idx, all_poses)
                if min(distances_m) > 1.0:
                    self._estimates.pop(idx)
                    break

            # Restart the loop if we deleted an estimate
            if len(all_poses) != len(self._estimates):
                continue

            # If there are still too many estimates, filter the "worst" estimate
            outlier_idx = identify_worst_outlier(all_poses)
            self._estimates.pop(outlier_idx)

    @property
    def all_estimates(self) -> list[PoseEstimate3D]:
        """Retrieve the list of all pose estimates forming the average."""
        return self._estimates

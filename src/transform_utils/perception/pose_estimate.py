"""Define dataclasses to represent the estimated poses of objects."""

from __future__ import annotations

from dataclasses import dataclass

from transform_utils.kinematics import Pose3D
from transform_utils.math.average_3d import compute_average_pose
from transform_utils.math.distances import compute_distances_to_others_m, identify_worst_outlier


@dataclass
class PoseEstimate3D:
    """A single 3D pose estimate with an associated confidence."""

    pose: Pose3D
    confidence: float = 0.0


class AveragedPoseEstimate3D:
    """Aggregate noisy pose estimates by averaging.

    Supports either unlimited history or a sliding window of the latest N estimates.
    """

    def __init__(self, max_estimates: int | None = None) -> None:
        """Initialize the averaged pose estimate.

        :param max_estimates: Maximum number of estimates to retain (None for unlimited).
        """
        self.max_estimates: int | None = max_estimates
        self._estimates: list[PoseEstimate3D] = []

    def reset(self) -> None:
        """Clear all stored pose estimates."""
        self._estimates.clear()

    def update(self, new_pose: Pose3D, confidence: float = 0.0) -> None:
        """Add a new pose estimate and prune if exceeding max_estimates.

        :param new_pose: New 3D pose to integrate into the average
        :param confidence: Confidence associated with the new pose (defaults to 0.0)
        """
        self._estimates.append(PoseEstimate3D(new_pose, confidence))

        # Enforce the sliding window until we're under the limit, if we have a limit
        while self.max_estimates is not None and len(self._estimates) > self.max_estimates:
            self._estimates.pop(0)

    @property
    def pose(self) -> Pose3D | None:
        """Return the current averaged pose, or None if no estimates are available."""
        if not self._estimates:
            return None

        poses = [e.pose for e in self._estimates]
        return compute_average_pose(poses, weights=None)  # Ignore confidences when averaging

    @property
    def known(self) -> bool:
        """Return True if any stored estimate has positive confidence."""
        return any(e.confidence > 0.0 for e in self._estimates)

    @property
    def all_estimates(self) -> list[PoseEstimate3D]:
        """Return a copy of all stored pose estimates."""
        return list(self._estimates)

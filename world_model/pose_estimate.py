"""Define a dataclass to represent an estimated object pose."""

from __future__ import annotations

from dataclasses import dataclass

from transform_utils.kinematics import Pose3D
from transform_utils.math.average_3d import compute_average_pose


@dataclass
class PoseEstimate3D:
    """An object pose estimate with an associated confidence value."""

    pose: Pose3D
    confidence: float = 0.0  # Confidence associated with this estimate


class AggregatePoseEstimate3D:
    """An object pose estimate based on multiple aggregated estimates."""

    def __init__(self) -> None:
        """Initialize the storage member variables of the PoseEstimate3D."""
        self._estimates: set[PoseEstimate3D]

    def update(self, new_estimate: PoseEstimate3D) -> None:
        """Update the combined estimate using the given pose estimate.

        :param new_estimate: New pose estimate to integrate into the combined estimate
        """
        self._estimates.add(new_estimate)

    @property
    def unknown(self) -> bool:
        """Return whether the estimated pose is currently unknown."""
        max_confidence = max([e.confidence for e in self._estimates], default=0.0)
        return max_confidence <= 0.0

    @property
    def estimated_pose(self) -> Pose3D | None:
        """Compute the aggregate estimated pose based on the stored estimates.

        Note: Currently uses an average of the stored estimates.

        :return: Estimated pose based on the available data
        """
        if not self._estimates:
            return None

        # For now, ignore the confidence of the pose estimates. It could be used to weight them
        poses = [est.pose for est in self._estimates]

        return compute_average_pose(poses, weights=None)

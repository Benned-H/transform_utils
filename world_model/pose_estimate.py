"""Define a dataclass to represent an estimated object pose."""

from __future__ import annotations

from dataclasses import dataclass

from transform_utils.kinematics import Pose3D


@dataclass
class PoseEstimate3D:
    """An object pose estimate with a confidence quantity."""

    estimated_pose: Pose3D | None = None  # None if no estimate has been received
    confidence: float = 0.0  # Confidence value associated with the stored pose estimate

    def update_estimate(self, new_pose: Pose3D, new_confidence: float) -> None:
        """Update the stored estimate based on the given pose estimate.

        :param new_pose: A new pose estimate to integrate into the stored estimate
        :param new_confidence: Confidence value associated with the given pose
        """
        if new_confidence > self.confidence:  # Keep the most confident of the estimates
            self.estimated_pose = new_pose
            self.confidence = new_confidence

    @property
    def unknown(self) -> bool:
        """Return whether the estimated pose is currently unknown."""
        return (self.estimated_pose is None) or (self.confidence <= 0.0)

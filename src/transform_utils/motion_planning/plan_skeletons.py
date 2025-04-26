"""Define dataclasses to represent skeletons of motion plans to be recomputed online."""

from __future__ import annotations

from dataclasses import dataclass

from transform_utils.kinematics import Configuration, Pose3D


@dataclass
class ManipulationPlanSkeleton:
    """A skeleton for an object-relative manipulation plan to be recomputed online."""

    robot_name: str  # Name of the robot to be used to execute the plan
    manipulator_name: str  # Name of the relevant manipulator
    target: Pose3D | Configuration  # Target end-effector pose w.r.t. object (or full-arm config.)


@dataclass
class NavigateToPosePlanSkeleton:
    """A skeleton for a navigation plan to a particular pose, to be recomputed online."""

    robot_name: str  # Name of the robot to be used to execute the plan
    target_base_pose: Pose3D  # Target robot base pose of the navigation plan

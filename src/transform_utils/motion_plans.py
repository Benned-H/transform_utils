"""Define dataclasses to represent motion plans to be recomputed online."""

from dataclasses import dataclass

from transform_utils.kinematics import Pose3D


@dataclass
class ManipulationPlanSkeleton:
    """A skeleton for an object-centric manipulation plan to be recomputed online."""

    robot_name: str  # Name of the robot to be used to execute the plan
    manipulator_name: str  # Name of the relevant manipulator
    target_o_ee: Pose3D  # Target end-effector pose in the object frame


@dataclass
class NavigationPlanSkeleton:
    """A skeleton for a navigation plan to be recomputed online."""

    robot_name: str  # Name of the robot to be used to execute the plan
    target_base_pose: Pose3D  # Target robot base pose of the motion plan

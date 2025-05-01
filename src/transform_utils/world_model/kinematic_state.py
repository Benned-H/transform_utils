"""Define a dataclass to represent the kinematic state of an object-centric world."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from transform_utils.filesystem.load_from_yaml import load_object_poses, load_robot_base_poses
from transform_utils.kinematics import Configuration, Pose3D


@dataclass
class KinematicState:
    """A kinematic state of a partially observable object-centric world.

    Here, "partially observable" means that object poses may be unknown (represented as None). We
        assume that all robot states (i.e., base poses and configurations) are known.
    """

    object_poses: dict[str, Pose3D | None]  # 3D poses of known objects (None if pose unknown)
    robot_base_poses: dict[str, Pose3D]  # Base poses of robots in the world
    robot_configurations: dict[str, Configuration]  # Configurations of the robots in the world

    @classmethod
    def from_yaml(cls, yaml_path: Path) -> KinematicState:
        """Construct a KinematicState instance using data from the given YAML file.

        :param yaml_path: YAML file containing data representing the kinematic state
        :return: Constructed KinematicState instance
        """
        obj_poses = load_object_poses(yaml_path)
        base_poses = load_robot_base_poses(yaml_path)
        robot_configs = {robot_name: {} for robot_name in base_poses}  # Default: No configurations

        return KinematicState(
            object_poses=obj_poses,
            robot_base_poses=base_poses,
            robot_configurations=robot_configs,
        )

    def get_object_pose(self, obj_name: str) -> Pose3D | None:
        """Retrieve the pose of the named object.

        :param obj_name: Name of an object in the world
        :return: Pose of the object (if pose is known), or None (if pose is unknown)
        :raises: KeyError, if an invalid object name is given
        """
        if obj_name not in self.object_poses:
            error = f"Cannot get pose of unknown object: '{obj_name}'."
            raise KeyError(error)

        return self.object_poses[obj_name]

    def set_object_pose(self, obj_name: str, new_pose: Pose3D) -> None:
        """Set the pose of the named object to the given pose.

        :param obj_name: Name of the object assigned the given pose
        :param new_pose: New 3D pose of the object
        :raises: KeyError, if an invalid object name is given
        """
        if obj_name not in self.object_poses:
            error = f"Cannot set pose of unknown object: '{obj_name}'."
            raise KeyError(error)

        self.object_poses[obj_name] = new_pose

    def get_robot_base_pose(self, robot_name: str) -> Pose3D:
        """Retrieve the base pose of the named robot.

        :param robot_name: Name of a robot
        :return: Base pose of the robot
        :raises: KeyError, if an invalid robot name is given
        """
        if robot_name not in self.robot_base_poses:
            error = f"Cannot get base pose of unknown robot: '{robot_name}'."
            raise KeyError(error)

        return self.robot_base_poses[robot_name]

    def set_robot_base_pose(self, robot_name: str, new_pose: Pose3D) -> None:
        """Set the base pose of the named robot to the given pose.

        :param robot_name: Name of the robot assigned the given base pose
        :param new_pose: New base pose of the robot
        :raises: KeyError, if an invalid robot name is given
        """
        if robot_name not in self.robot_base_poses:
            error = f"Cannot set base pose of unknown robot: '{robot_name}'."
            raise KeyError(error)

        self.robot_base_poses[robot_name] = new_pose

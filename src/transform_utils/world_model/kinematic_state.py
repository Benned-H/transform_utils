"""Define a dataclass to represent the kinematic state of an object-centric world."""

from __future__ import annotations

from dataclasses import dataclass

from transform_utils.kinematics import Configuration, Pose2D, Pose3D


@dataclass
class KinematicState:
    """A kinematic state of a partially observable object-centric world.

    Here, "partially observable" means that object poses may be unknown (represented as None). We
        assume that all robot states (i.e., base poses and configurations) are known.
    """

    object_poses: dict[str, Pose3D | None]  # 3D poses of known objects (None if pose unknown)
    robot_base_poses: dict[str, Pose2D]  # Base poses of robots in the world
    robot_configurations: dict[str, Configuration]  # Configurations of the robots in the world
    object_is_static: dict[str, bool]  # Maps object names to whether they're static (cannot move)

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

        # Don't allow changing a static object's pose (if the pose is already known)
        curr_pose = self.object_poses[obj_name]
        if self.object_is_static[obj_name] and curr_pose is not None:
            error = f"Cannot change the pose of static object '{obj_name}'."
            raise ValueError(error)

        self.object_poses[obj_name] = new_pose

    def get_robot_base_pose(self, robot_name: str) -> Pose2D:
        """Retrieve the base pose of the named robot.

        :param robot_name: Name of a robot
        :return: Base pose of the robot
        :raises: KeyError, if an invalid robot name is given
        """
        if robot_name not in self.robot_base_poses:
            error = f"Cannot get base pose of unknown robot: '{robot_name}'."
            raise KeyError(error)

        return self.robot_base_poses[robot_name]

    def set_robot_base_pose(self, robot_name: str, new_pose: Pose2D) -> None:
        """Set the base pose of the named robot to the given pose.

        :param robot_name: Name of the robot assigned the given base pose
        :param new_pose: New 2D base pose of the robot
        :raises: KeyError, if an invalid robot name is given
        """
        if robot_name not in self.robot_base_poses:
            error = f"Cannot set base pose of unknown robot: '{robot_name}'."
            raise KeyError(error)

        self.robot_base_poses[robot_name] = new_pose

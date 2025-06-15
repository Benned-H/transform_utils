"""Represent the geometric state of the environment as a kinematic tree."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from transform_utils.filesystem.load_from_yaml import load_object_poses, load_robot_base_poses
from transform_utils.kinematics import Configuration, Pose3D
from transform_utils.states.collision_models import CollisionModel
from transform_utils.states.landmarks_2d import Landmarks2D


class KinematicTree:
    """A tree of coordinate frames specifying relative poses between entities."""

    def __init__(self) -> None:
        """Initialize the kinematic tree's member variables as empty."""
        self.frames: dict[str, Pose3D] = {}  # Maps frame names to their relative poses
        self.children: dict[str, set[str]] = {}  # Maps frame names to their children frames

        # Maps frame names to their (optional) attached collision geometry
        self.collision_models: dict[str, CollisionModel] = {}

        # Record which frames correspond to objects or robot base poses
        self.object_names: set[str] = set()  # Object frame names: f"{object_name}"
        self.robot_names: set[str] = set()  # Base pose frame names: f"{robot_name}_base_pose"

        # Store robot configurations to represent actuated joints in the kinematic tree
        self.robot_configs: dict[str, Configuration]  # Configurations of robots in the world

        # Store 2D landmarks used as navigation waypoints
        self.landmarks: Landmarks2D = Landmarks2D()

    @classmethod
    def from_yaml(cls, yaml_path: Path) -> KinematicTree:
        """Construct a KinematicTree instance using data from the given YAML file.

        :param yaml_path: YAML file containing data representing the kinematic state
        :return: Constructed KinematicTree instance
        """
        tree = KinematicTree()

        for obj_name, obj_pose in load_object_poses(yaml_path).items():
            tree.set_object_pose(obj_name, obj_pose)

        for robot_name, robot_base_pose in load_robot_base_poses(yaml_path).items():
            tree.set_robot_base_pose(robot_name, robot_base_pose)

            tree.robot_configs[robot_name] = {}  # Default: No robot configurations

        tree.landmarks = Landmarks2D.from_yaml(yaml_path)

        return tree

    @property
    def object_poses(self) -> dict[str, Pose3D]:
        """Create and return a dictionary mapping object names to their 3D poses."""
        return {obj_name: self.frames[obj_name] for obj_name in self.object_names}

    @property
    def robot_base_poses(self) -> dict[str, Pose3D]:
        """Create and return a dictionary mapping robot names to their base poses."""
        return {r_name: self.frames[f"{r_name}_base_pose"] for r_name in self.robot_names}

    def get_parent_frame(self, child_frame: str) -> str | None:
        """Retrieve the name of the parent frame of a frame.

        :param child_frame: Frame whose parent frame is retrieved
        :return: Name of the relative frame of the child frame (None if parent frame is unknown)
        """
        child_frame_pose = self.frames.get(child_frame)
        return None if child_frame_pose is None else child_frame_pose.ref_frame

    def update_frame(self, frame_name: str, pose: Pose3D) -> None:
        """Update the named frame with the given relative pose.

        :param frame_name: Name of the reference frame added or updated
        :param pose: Relative pose of the frame
        """
        prev_parent_frame = self.get_parent_frame(frame_name)
        if prev_parent_frame is not None:  # Remove this frame from its previous parent's children
            self.children[prev_parent_frame].remove(frame_name)

        self.frames[frame_name] = pose
        self.children[frame_name] = self.children.get(frame_name, set())  # Initialize children
        self.children[pose.ref_frame].add(frame_name)  # Add this frame to its parent's children

    def set_collision_model(self, frame_name: str, collision_model: CollisionModel) -> None:
        """Set the collision geometry attached to the named frame.

        :param frame_name: Name of the frame to which the collision model is attached
        :param collision_model: Rigid-body collision geometry (geometric primitive or mesh)
        """
        assert frame_name in self.frames, f"Unknown frame name: '{frame_name}'."

        self.collision_models[frame_name] = collision_model

    def set_object_pose(self, obj_name: str, new_pose: Pose3D) -> None:
        """Set the pose of the named object to the given pose.

        :param obj_name: Name of the object assigned the given pose
        :param new_pose: New 3D pose of the object
        """
        self.update_frame(obj_name, new_pose)
        self.object_names.add(obj_name)

    def get_object_pose(self, obj_name: str) -> Pose3D:
        """Retrieve the pose of the named object.

        :param obj_name: Name of an object in the world
        :return: Pose of the object (if pose is known), or None (if pose is unknown)
        :raises: KeyError, if an invalid object name is given
        """
        if obj_name not in self.object_names or obj_name not in self.frames:
            raise KeyError(f"Cannot get pose of unknown object: '{obj_name}'.")

        return self.frames[obj_name]

    def set_robot_base_pose(self, robot_name: str, new_pose: Pose3D) -> None:
        """Set the base pose of the named robot to the given pose.

        :param robot_name: Name of the robot assigned the given base pose
        :param new_pose: New base pose of the robot
        """
        self.update_frame(f"{robot_name}_base_pose", new_pose)
        self.robot_names.add(robot_name)

    def get_robot_base_pose(self, robot_name: str) -> Pose3D:
        """Retrieve the base pose of the named robot.

        :param robot_name: Name of a robot
        :return: Base pose of the robot
        :raises: KeyError, if an invalid robot name is given
        """
        if robot_name not in self.robot_names or f"{robot_name}_base_pose" not in self.frames:
            raise KeyError(f"Cannot get base pose of unknown robot: '{robot_name}'.")

        return self.frames[f"{robot_name}_base_pose"]

    def convert_poses_to_yaml(self) -> str:
        """Convert the robot and object poses in the kinematic tree into a YAML string.

        :return: String representation of the current poses in YAML
        """
        yaml_data: dict[str, dict[str, Any]] = {"object_poses": {}, "robot_base_poses": {}}

        for obj_name, obj_pose in self.object_poses.items():
            yaml_data["object_poses"][obj_name] = obj_pose.to_yaml_dict()

        for robot_name, robot_base_pose in self.robot_base_poses.items():
            yaml_data["robot_base_poses"][robot_name] = robot_base_pose.to_yaml_dict()
        return yaml.dump(yaml_data, sort_keys=True, default_flow_style=True)

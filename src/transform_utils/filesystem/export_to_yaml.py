"""Define functions for exporting various data structures to YAML."""

from typing import Any

import yaml

from transform_utils.world_model.kinematic_state import KinematicState


def output_poses_to_yaml(state: KinematicState) -> str:
    """Convert the poses represented in the given kinematic state into a YAML string.

    Objects without a known pose are simply skipped in the YAML output.

    :param state: Kinematic state including object poses and robot base poses
    :return: String representation of the state's poses in YAML
    """
    yaml_data: dict[str, dict[str, Any]] = {"object_poses": {}, "robot_base_poses": {}}

    for obj_name, obj_pose in state.object_poses.items():
        if obj_pose is None:
            continue  # Skip objects with unknown poses
        yaml_data["object_poses"][obj_name] = obj_pose.to_yaml_dict()

    for robot_name, robot_base_pose in state.robot_base_poses.items():
        yaml_data["robot_base_poses"][robot_name] = robot_base_pose.to_yaml_dict()

    return yaml.dump(yaml_data, sort_keys=True, default_flow_style=True)

"""Define functions for exporting various data structures to YAML."""

from typing import Any

import yaml

from transform_utils.filesystem.load_from_yaml_ros import EnvironmentModel


def output_poses_to_yaml(env: EnvironmentModel) -> str:
    """Convert the robot base poses and object poses of an environment model into a YAML string.

    :param env: Environment model containing robot base poses and objects
    :return: String representation of the environment poses in YAML
    """
    yaml_data: dict[str, dict[str, Any]] = {"robot_base_poses": {}, "objects": {}}

    for robot_name, robot_base_pose in env.base_poses.items():
        yaml_data["robot_base_poses"][robot_name] = robot_base_pose.to_yaml_dict()

    for obj_name, obj_model in env.objects.items():
        if obj_model.pose is not None:
            yaml_data["objects"][obj_name] = obj_model.pose.to_yaml_dict()

    return yaml.dump(yaml_data, sort_keys=True, default_flow_style=True)

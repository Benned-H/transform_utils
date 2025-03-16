"""Define functions for exporting object and environment models to YAML."""

from typing import Any

import yaml

from transform_utils.world_model.load_from_yaml import EnvironmentModel


def get_object_poses_yaml(env: EnvironmentModel) -> str:
    """Convert the object poses in an environment model into a YAML string.

    :param env: Environment model containing robot base poses and objects
    :return: String representation of the environment's object poses in YAML
    """
    yaml_data: dict[str, Any] = {}
    yaml_data["objects"] = {}

    for obj_name, object_data in env.objects.items():
        if object_data.pose is not None:
            yaml_data["objects"][obj_name] = {
                "pose": object_data.pose.to_list(),
                "frame": object_data.pose.ref_frame,
            }
    return yaml.dump(yaml_data, sort_keys=True)

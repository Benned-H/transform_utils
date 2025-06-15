"""Define functions for exporting various data structures to YAML."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


def output_yaml_data_to_path(data: dict[str, Any], yaml_path: Path) -> bool:
    """Output the given dictionary of YAML data to the given path.

    :param data: Dictionary of YAML data to be output to file
    :param yaml_path: Path to the created YAML file
    :return: True if output succeeded, else False
    """
    yaml_string = yaml.dump(data, sort_keys=True, default_flow_style=True)

    with yaml_path.open(mode="w") as yaml_file:
        yaml_file.write(yaml_string)
        yaml_file.close()

    return yaml_path.exists()

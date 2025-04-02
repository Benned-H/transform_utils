"""Define utility functions for working with ROS and ROS messages."""

from __future__ import annotations

from pathlib import Path

import rospy
from rospkg import RosPack


def resolve_package_path(relative_path: str) -> Path | None:
    """Resolve a filepath relative to a ROS package.

    For example,
      Input (relative): spot_skills/launch/demo.launch
      Output (absolute): /home/Documents/catkin_ws/src/spot_skills/launch/demo.launch

    :param relative_path: Path to a file, beginning with a ROS package
    :return: Path object containing the full absolute path (or None if path doesn't exist)
    """
    package_name, relative_path = relative_path.split("/", maxsplit=1)

    rospack = RosPack()
    package_path = rospack.get_path(package_name)

    full_path = Path(package_path, relative_path)

    if not full_path.exists():
        rospy.logerr(f"Could not resolve package-relative filepath: {relative_path}.")
        return None

    return full_path

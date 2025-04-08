"""Define utility functions to log messages and errors in a ROS-agnostic manner."""

import rospy


def log_info(message: str) -> None:
    """Log a message to the available information output channels.

    :param message: Message to be logged as information
    """
    rospy.loginfo(message)  # TODO: Define ROS-free version


def log_error(message: str) -> None:
    """Log a message to the available error output channels.

    :param message: Message to be logged as an error
    """
    rospy.logerr(message)  # TODO: Define ROS-free version

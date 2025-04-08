"""Define utility functions to log messages and errors in a ROS-agnostic manner."""

try:
    import rospy

    ROS_PRESENT = True
except ModuleNotFoundError:
    ROS_PRESENT = False


def log_info(message: str) -> None:
    """Log a message to the available information output channels.

    :param message: Message to be logged as information
    """
    if ROS_PRESENT:
        rospy.loginfo(message)
    else:
        print(message)


def log_error(message: str) -> None:
    """Log a message to the available error output channels.

    :param message: Message to be logged as an error
    """
    if ROS_PRESENT:
        rospy.logerr(message)
    else:
        print(message)

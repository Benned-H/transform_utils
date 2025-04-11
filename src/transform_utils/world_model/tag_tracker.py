"""Define a class to manage the tracking of detected AprilTag poses and dependent objects."""

from pathlib import Path

from transform_utils.filesystem.load_from_yaml import load_apriltag_relative_transforms_from_yaml


class TagTracker:
    """A class that stores and re-publishes the poses of detected AR tags."""

    def __init__(self, tags_yaml: Path) -> None:
        """Initialize the TagTracker by populating its list of known AprilTags.

        :param tags_yaml: Path to a YAML file specifying AprilTag IDs and sizes
        """


#     def __init__(self) -> None:
#         """Initialize the TagTracker by retrieving expected tag sizes from ROS parameters."""
#         self.tag_poses: dict[int, Pose3D] = {}

#         self._body_cam_marker_size_cm = rospy.get_param("/spot/ar/body_cam_marker_size_cm")
#         self._hand_cam_marker_size_cm = rospy.get_param("/spot/ar/hand_cam_marker_size_cm")

#     def marker_callback(self, markers_msg: AlvarMarkers) -> None:
#         """Store updated tag poses from detected AR markers.

#         :param markers_msg: Message containing a list of tag detections
#         """
#         for marker in markers_msg.markers:
#             rospy.loginfo(f"Detected Marker ID: {marker.id} with confidence {marker.confidence}.")
#             self.tag_poses[marker.id] = pose_from_msg(marker.pose)

#         # TODO: Filter which poses are trusted based on the actual tag size and camera name

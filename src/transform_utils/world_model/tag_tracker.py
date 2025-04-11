"""Define a class to manage the tracking of detected AprilTag poses and dependent objects."""

from pathlib import Path

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers

from transform_utils.kinematics_ros import pose_from_msg
from transform_utils.world_model.april_tag import AprilTagSystem


class TagTracker:
    """A class that stores and re-publishes the poses of detected AR tags."""

    def __init__(self) -> None:
        """Initialize the TagTracker with empty member variables."""
        self.tag_system: AprilTagSystem | None = None  # Known AprilTags and their detecting cameras

        self.marker_subs: list[rospy.Subscriber] = []

    def populate_from_yaml(self, yaml_path: Path) -> None:
        """Populate the TagTracker's tags, cameras, and ROS subscribers from YAML.

        :param yaml_path: Path to a YAML file specifying AprilTag data
        """
        self.tag_system = AprilTagSystem.from_yaml(yaml_path)

        # Populate the ROS subscribers based on the imported tags and camera names
        for camera_name in self.tag_system.camera_detects_tags:
            self.marker_subs.append(
                rospy.Subscriber(
                    f"{self.tag_system.camera_topic_prefix}{camera_name}",
                    AlvarMarkers,
                    callback=self.marker_callback,
                    callback_args=camera_name,
                    queue_size=5,
                ),
            )

    def marker_callback(self, markers_msg: AlvarMarkers, camera_name: str) -> None:
        """Handle new AR marker detections from the named camera.

        :param markers_msg: Message containing a list of tag detections
        :param camera_name: Name of the camera source of the detections
        """
        assert camera_name in self.tag_system.camera_detects_tags, (
            f"Unrecognized camera name: {camera_name}."
        )

        for marker in markers_msg.markers:
            if marker.id not in self.tag_system.camera_detects_tags[camera_name]:
                continue  # Move to the next marker detection

            tag_size_cm = self.tag_system.tags[marker.id].size_cm

            rospy.loginfo(
                f"Marker ID {marker.id} (size {tag_size_cm} cm) was detected by camera "
                f"'{camera_name}' with confidence {marker.confidence}.",
            )

            self.tag_system.tags[marker.id].pose = pose_from_msg(marker.pose)  # Update tag's pose

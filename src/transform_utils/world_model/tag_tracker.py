"""Define a class to manage the tracking of detected AprilTag poses and dependent objects."""

import threading

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers

from transform_utils.kinematics_ros import pose_from_msg
from transform_utils.transform_manager import TransformManager
from transform_utils.world_model.april_tag import AprilTagSystem


class TagTracker:
    """A class that stores and re-publishes the poses of detected AR tags."""

    def __init__(self, tag_system: AprilTagSystem) -> None:
        """Initialize the TagTracker for a system of AprilTags and cameras.

        :param tag_system: System of known AprilTags and cameras to detect them
        """
        self.tag_system = tag_system

        self.marker_subs: list[rospy.Subscriber] = []

        # Populate the ROS subscribers based on the imported tags and camera names
        for camera_name, camera in self.tag_system.cameras.items():
            if camera.detects_single_tags:
                topic_name = f"{self.tag_system.camera_topic_prefix}/{camera_name}/single"
                self.marker_subs.append(
                    rospy.Subscriber(
                        topic_name,
                        AlvarMarkers,
                        callback=self.marker_callback,
                        callback_args=topic_name,
                        queue_size=5,
                    ),
                )

            if camera.detects_bundles:
                topic_name = f"{self.tag_system.camera_topic_prefix}/{camera_name}/bundle"
                self.marker_subs.append(
                    rospy.Subscriber(
                        topic_name,
                        AlvarMarkers,
                        callback=self.marker_callback,
                        callback_args=topic_name,
                        queue_size=5,
                    ),
                )

        self._tf_publisher_thread = threading.Thread(target=self._publish_frames_loop)
        self._tf_publisher_thread.daemon = True  # Thread exits when main process does
        self._tf_publisher_thread.start()

    def marker_callback(self, markers_msg: AlvarMarkers, camera_name: str, bundle: bool) -> None:
        """Handle new AR marker detections from the named camera.

        :param markers_msg: Message containing a list of tag detections
        :param camera_name: Name of the camera source of the detections
        :param bundle: Whether this topic corresponds to bundled markers
        """
        if camera_name not in self.tag_system.cameras:
            rospy.logwarn(f"Unrecognized camera name: {camera_name}.")

        for marker in markers_msg.markers:
            if marker.id not in self.tag_system.camera_detects_tags[camera_name]:
                continue  # Move to the next marker detection

            actual_tag = self.tag_system.tags[marker.id]
            if actual_tag.in_bundle != bundle:
                continue  # Ensure that single-tag detections aren't used for bundled markers

            rospy.loginfo(
                f"Marker ID {marker.id} (size {actual_tag.size_cm} cm) was detected by camera "
                f"'{camera_name}' with confidence {marker.confidence}.",
            )

            self.tag_system.tags[marker.id].pose = pose_from_msg(marker.pose)  # Update tag's pose

    def _publish_frames_loop(self) -> None:
        """Publish the known AprilTags' poses (and dependent objects') frames to /tf."""
        try:
            rate_hz = rospy.Rate(TransformManager.loop_hz)
            while not rospy.is_shutdown():
                for tag in self.tag_system.tags.values():
                    if tag.pose is not None:
                        TransformManager.broadcast_transform(tag.frame_name, tag.pose)

                        for obj_name, relative_pose in tag.relative_frames.items():
                            TransformManager.broadcast_transform(obj_name, relative_pose)
                rate_hz.sleep()

        except rospy.ROSInterruptException as ros_exc:
            rospy.logwarn(f"[TagTracker._publish_frames_loop] {ros_exc}")

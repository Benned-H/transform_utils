"""Define a class to manage the tracking of detected AprilTag poses and dependent objects."""

import threading
from dataclasses import dataclass
from pathlib import Path

import rospy
import yaml
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

from transform_utils.kinematics import DEFAULT_FRAME
from transform_utils.kinematics_ros import pose_from_msg
from transform_utils.perception.april_tag import AprilTagSystem
from transform_utils.transform_manager import TransformManager


@dataclass
class MarkerCallbackArgs:
    """A data structure organizing arguments of the TagTracker.marker_callback() method."""

    camera_name: str  # Name of the camera source of a tag detection
    bundle: bool  # Whether the ROS topic contains bundle-based detections


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
                callback_args = MarkerCallbackArgs(camera_name, bundle=False)

                self.marker_subs.append(
                    rospy.Subscriber(
                        f"{self.tag_system.camera_topic_prefix}/{camera_name}/single",
                        AlvarMarkers,
                        callback=self.marker_callback,
                        callback_args=callback_args,
                        queue_size=5,
                    ),
                )

            if camera.detects_bundles:
                callback_args = MarkerCallbackArgs(camera_name, bundle=True)

                self.marker_subs.append(
                    rospy.Subscriber(
                        f"{self.tag_system.camera_topic_prefix}/{camera_name}/bundle",
                        AlvarMarkers,
                        callback=self.marker_callback,
                        callback_args=callback_args,
                        queue_size=5,
                    ),
                )

        self._output_srv = rospy.Service("~output_to_yaml", Trigger, self.handle_output_to_yaml)

        self._tf_publisher_thread = threading.Thread(target=self._publish_frames_loop)
        self._tf_publisher_thread.daemon = True  # Thread exits when main process does
        self._tf_publisher_thread.start()

    def marker_callback(self, markers_msg: AlvarMarkers, args: MarkerCallbackArgs) -> None:
        """Handle new AR marker detections from the named camera.

        :param markers_msg: Message containing a list of tag detections
        :param args: Data structure organizing arguments to the callback
        """
        if args.camera_name not in self.tag_system.cameras:
            rospy.logwarn(f"Unrecognized camera name: '{args.camera_name}'.")
            return

        for marker in markers_msg.markers:
            if marker.id not in self.tag_system.camera_detects_tags[args.camera_name]:
                continue  # Move to the next marker detection

            tag = self.tag_system.tags[marker.id]
            if tag.in_bundle != args.bundle:
                continue  # Ensure that single-tag detections aren't used for bundled markers

            rospy.loginfo_throttle(
                period=5,
                msg=f"Marker {marker.id} ({tag.size_cm} cm) was detected by '{args.camera_name}'.",
            )

            pose = pose_from_msg(marker.pose)
            pose.ref_frame = marker.header.frame_id
            pose_w_t = TransformManager.convert_to_frame(pose, DEFAULT_FRAME)

            self.tag_system.tags[marker.id].pose = pose_w_t  # Update tag's pose w.r.t. world

    def handle_output_to_yaml(self, _: TriggerRequest) -> TriggerResponse:
        """Handle a request to output the current object pose estimates to YAML.

        :return: ROS message specifying whether the output process was successful
        """
        yaml_path = Path(rospy.get_param("~yaml_output_path"))
        if yaml_path.suffix not in {".yaml", ".yml"}:
            message = f"Cannot output YAML to path {yaml_path}; invalid file suffix."
            return TriggerResponse(success=False, message=message)

        poses_data = {"object_poses": {}, "default_frame": DEFAULT_FRAME}
        for tag in self.tag_system.tags.values():
            for object_name in tag.relative_frames:
                pose_w_o = TransformManager.lookup_transform(object_name, DEFAULT_FRAME)
                poses_data["object_poses"][object_name] = pose_w_o.to_list()

        yaml_string = yaml.dump(poses_data, sort_keys=True, default_flow_style=True)

        with yaml_path.open(mode="w") as yaml_file:
            yaml_file.write(yaml_string)
            yaml_file.close()

        success = yaml_path.exists()
        message = f"Wrote to file: {yaml_path}." if success else f"Could not write to {yaml_path}."

        return TriggerResponse(success=success, message=message)

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

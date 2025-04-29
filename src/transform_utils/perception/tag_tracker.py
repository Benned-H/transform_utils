"""The TagTracker class manages the tracking of detected AprilTags and any dependent objects."""

from __future__ import annotations

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
from transform_utils.perception.pose_estimate import AveragedPoseEstimate3D
from transform_utils.transform_manager import TransformManager


@dataclass
class MarkerCallbackArgs:
    """A data structure organizing arguments of the TagTracker.marker_callback() method."""

    camera_name: str  # Name of the camera source of a tag detection
    bundle: bool  # Whether the ROS topic contains bundle-based detections


class TagTracker:
    """Tracks AR tag detections, aggregates the last N poses per tag, and republishes averages."""

    def __init__(self, tag_system: AprilTagSystem, window_size: int | None = 15) -> None:
        """Initialize the TagTracker for a system of AprilTags and cameras.

        :param tag_system: System of known AprilTags and cameras to detect them
        :param window_size: Number of recent pose estimates to track per tag (None = infinite)
        """
        self.tag_system = tag_system
        self.window_size = window_size

        # Pose estimators per tag ID
        self._estimators: dict[int, AveragedPoseEstimate3D] = {
            tag.id: AveragedPoseEstimate3D(max_estimates=self.window_size)
            for tag in self.tag_system.tags.values()
        }

        # Subscribers for single tags and bundles
        self.marker_subs: list[rospy.Subscriber] = []
        for camera_name, camera in self.tag_system.cameras.items():
            if camera.detects_single_tags:
                args = MarkerCallbackArgs(camera_name, bundle=False)
                topic = f"{self.tag_system.camera_topic_prefix}/{camera_name}/single"
                single_sub = rospy.Subscriber(
                    topic,
                    AlvarMarkers,
                    callback=self.marker_callback,
                    callback_args=args,
                    queue_size=5,
                )
                self.marker_subs.append(single_sub)

            if camera.detects_bundles:
                args = MarkerCallbackArgs(camera_name, bundle=True)
                topic = f"{self.tag_system.camera_topic_prefix}/{camera_name}/bundle"
                bundle_sub = rospy.Subscriber(
                    topic,
                    AlvarMarkers,
                    callback=self.marker_callback,
                    callback_args=args,
                    queue_size=5,
                )
                self.marker_subs.append(bundle_sub)

        self._output_srv = rospy.Service("~output_to_yaml", Trigger, self.handle_output_to_yaml)

        # Thread to publish TF frames
        self._tf_publisher_thread = threading.Thread(target=self._publish_frames_loop)
        self._tf_publisher_thread.daemon = True  # Thread exits when main process does
        self._tf_publisher_thread.start()

    def marker_callback(self, markers_msg: AlvarMarkers, args: MarkerCallbackArgs) -> None:
        """Update pose estimates based on AR marker detections from the named camera.

        :param markers_msg: Message containing a list of tag detections
        :param args: Data structure organizing arguments to the callback
        """
        if args.camera_name not in self.tag_system.cameras:
            rospy.logwarn(f"Unrecognized camera name: '{args.camera_name}'.")
            return

        for marker in markers_msg.markers:
            camera_tags = self.tag_system.camera_detects_tags[args.camera_name]
            if marker.id not in camera_tags:
                continue  # Camera doesn't detect this tag; move to next detection

            tag = self.tag_system.tags[marker.id]
            if tag.in_bundle != args.bundle:
                continue  # Ensure that single-tag detections aren't used for bundled markers

            rospy.loginfo_throttle(
                period=3,
                msg=f"Marker {marker.id} ({tag.size_cm} cm) was detected by '{args.camera_name}'.",
            )

            raw_pose = pose_from_msg(marker.pose)
            raw_pose.ref_frame = marker.header.frame_id
            pose_w_t = TransformManager.convert_to_frame(raw_pose, DEFAULT_FRAME)
            if pose_w_t is None:
                continue

            # Update estimator and store averaged pose
            estimator = self._estimators.get(marker.id)
            if estimator is None:
                continue
            estimator.update(pose_w_t)
            avg_pose = estimator.pose
            tag.pose = avg_pose  # None if not enough data yet

    def handle_output_to_yaml(self, _: TriggerRequest) -> TriggerResponse:
        """Dump the current estimated object poses to YAML.

        :return: ROS message specifying whether the export process was successful
        """
        yaml_path = Path(rospy.get_param("~yaml_output_path"))
        if yaml_path.suffix not in {".yaml", ".yml"}:
            return TriggerResponse(success=False, message=f"Invalid YAML file suffix: {yaml_path}")

        poses_data = {"object_poses": {}, "default_frame": DEFAULT_FRAME}
        for tag in self.tag_system.tags.values():
            if tag.pose is None:
                continue

            for object_name in tag.relative_frames:
                pose_w_o = TransformManager.lookup_transform(object_name, DEFAULT_FRAME)
                poses_data["object_poses"][object_name] = pose_w_o.to_list()

        yaml_string = yaml.dump(poses_data, sort_keys=True, default_flow_style=True)

        with yaml_path.open("w") as yaml_file:
            yaml_file.write(yaml_string)
            yaml_file.close()

        ok = yaml_path.exists()
        message = f"Wrote YAML to {yaml_path}." if ok else f"Failed to write to {yaml_path}."

        return TriggerResponse(success=ok, message=message)

    def _publish_frames_loop(self) -> None:
        """Continuously broadcast averaged tag and object frames to /tf."""
        rate_hz = rospy.Rate(TransformManager.loop_hz)
        try:
            while not rospy.is_shutdown():
                for tag in self.tag_system.tags.values():
                    avg_pose = self._estimators[tag.id].pose
                    if avg_pose is not None:
                        TransformManager.broadcast_transform(tag.frame_name, avg_pose)

                        for obj_name, relative_pose in tag.relative_frames.items():
                            TransformManager.broadcast_transform(obj_name, relative_pose)
                rate_hz.sleep()

        except rospy.ROSInterruptException as ros_exc:
            rospy.logwarn(f"[TagTracker] TF publish loop interrupted: {ros_exc}")

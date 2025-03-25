"""Define a class to manage setting and reading transforms from the /tf tree."""

from __future__ import annotations

from typing import TYPE_CHECKING

import rospy
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from transform_utils.kinematics_ros import pose_from_tf_stamped_msg, pose_to_tf_stamped_msg

if TYPE_CHECKING:
    from geometry_msgs.msg import TransformStamped

    from transform_utils.kinematics import Pose2D, Pose3D


class TransformManager:
    """A static class used to manage and read from the /tf tree."""

    loop_hz = 10.0  # Frequency (Hz) of any transform request/send loops
    lookup_timeout_s = 5.0  # Duration (seconds) after which to abandon a transform lookup

    # Delay initialization of TF2 objects until ROS is available
    _tf_broadcaster: TransformBroadcaster | None = None
    _tf_buffer: Buffer | None = None
    _tf_listener: TransformListener | None = None

    @staticmethod
    def init_node(node_name: str = "transform_manager") -> None:
        """Initialize a ROS node if this process does not yet have a ROS node.

        This method allows the TransformManager to initialize its transform
            listener as soon as it knows that ROS is available.

        :param node_name: Name of the node, defaults to "transform_manager"
        """
        if rospy.get_name() in ["", "/unnamed"]:
            rospy.init_node(node_name)
            rospy.loginfo(f"Initialized node with name '{rospy.get_name()}'")

        TransformManager.tf_listener()  # Ensure transform listener is initialized

    @staticmethod
    def tf_broadcaster() -> TransformBroadcaster:
        """Retrieve the transform broadcaster, initializing it if necessary.

        :return: Transform broadcaster used to send transforms to the TF2 server
        """
        if TransformManager._tf_broadcaster is None:
            TransformManager._tf_broadcaster = TransformBroadcaster()
        return TransformManager._tf_broadcaster

    @staticmethod
    def tf_buffer() -> Buffer:
        """Retrieve the transform buffer, initializing it if necessary.

        :return: Transform buffer used to store known transforms
        """
        if TransformManager._tf_buffer is None:
            TransformManager._tf_buffer = Buffer()
        return TransformManager._tf_buffer

    @staticmethod
    def tf_listener() -> TransformListener:
        """Retrieve the transform listener, initializing it if necessary.

        :return: Transform listener used to receive transforms from the TF2 server
        """
        if TransformManager._tf_listener is None:
            tf_buffer = TransformManager.tf_buffer()
            TransformManager._tf_listener = TransformListener(tf_buffer)
        return TransformManager._tf_listener

    @staticmethod
    def broadcast_transform(frame_name: str, pose: Pose3D) -> None:
        """Broadcast the given transform for the named frame into /tf.

        :param frame_name: Name of the reference frame to be updated
        :param pose: Transform of the frame relative to some other frame
        """
        tf_stamped_msg = pose_to_tf_stamped_msg(pose, frame_name)
        tf_stamped_msg.header.stamp = rospy.Time.now()

        TransformManager.tf_broadcaster().sendTransform(tf_stamped_msg)

    @staticmethod
    def lookup_transform(source_frame: str, target_frame: str, when: rospy.Time) -> Pose3D | None:
        """Look up the transform to convert from one frame to another using /tf.

        Frame notation: Frame of some data (d), source frame (s), target frame (t).

        Say our input data originates in the source frame: pose_s_d ("data in the source frame").
        This function outputs transform_t_s ("source relative to target"), which lets us compute:

            transform_t_s @ pose_s_d = pose_t_d ("data expressed in the target frame")

        :param source_frame: Frame where the data originated
        :param target_frame: Frame to which the data will be transformed
        :param when: Timestamp at which the relative transform is found
        :return: Pose3D representing transform (i.e., transform_t_s) or None (if lookup failed)
        """
        rate_hz = rospy.Rate(TransformManager.loop_hz)
        rate_hz.sleep()

        start_time_s = rospy.get_time()  # Start time as float seconds
        timeout_time_s = start_time_s + TransformManager.lookup_timeout_s

        tf_stamped_msg = None
        while (rospy.get_time() < timeout_time_s) and (not rospy.is_shutdown_requested()):
            try:
                tf_stamped_msg: TransformStamped = TransformManager.tf_buffer().lookup_transform(
                    target_frame=target_frame,
                    source_frame=source_frame,
                    time=when,
                    timeout=rate_hz.sleep_dur,
                )
                break
            except TransformException as t_exc:
                rospy.logwarn(
                    f"[TransformManager.lookup_transform] Looking for {source_frame} to "
                    f"{target_frame} at time {when.to_time():.2f} gave exception: {t_exc}",
                )
                rate_hz.sleep()

        if tf_stamped_msg is None:
            rospy.logerr(
                f"[TransformManager.lookup_transform] Could not look up transform "
                f"from {source_frame} to {target_frame} at time {when.to_time():.2f}",
            )
            return None

        pose_t_s = pose_from_tf_stamped_msg(tf_stamped_msg)

        assert target_frame == pose_t_s.ref_frame, (
            f"Expected result in '{target_frame}' but instead found '{pose_t_s.ref_frame}'"
        )

        return pose_t_s

    @staticmethod
    def convert_to_frame(pose_c_p: Pose2D | Pose3D, new_ref_frame: str) -> Pose3D:
        """Convert the given pose into a new reference frame.

        Frames: Frame implied by the pose (p), current ref. frame (c), new ref. frame (n)

        :param pose_c_p: Pose (frame p) w.r.t. its current reference frame (frame c)
        :param new_ref_frame: New reference frame (frame n) of the returned pose
        :return: Pose3D relative to the new reference frame (i.e., pose_n_p)
        """
        if isinstance(pose_c_p, Pose2D):
            pose_c_p = Pose3D.from_2d(pose_c_p)

        if pose_c_p.ref_frame == new_ref_frame:
            return pose_c_p

        now = rospy.Time.now()
        pose_n_c = TransformManager.lookup_transform(pose_c_p.ref_frame, new_ref_frame, now)

        return pose_n_c @ pose_c_p

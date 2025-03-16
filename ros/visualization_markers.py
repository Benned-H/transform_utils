"""Define functions to convert poses into RViz visualization markers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import rospy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker as MarkerMsg

from transform_utils.kinematics import DEFAULT_FRAME, Point3D, Pose3D
from transform_utils.kinematics_ros import point_to_msg
from transform_utils.transform_manager import TransformManager

if TYPE_CHECKING:
    from geometry_msgs.msg import Point as PointMsg

    from transform_utils.world_model.pose_estimate import AveragedPoseEstimate3D
    from transform_utils.world_model.world_objects import WorldObjects


@dataclass
class RGBAxesConfig:
    """A dataclass storing default settings for RGB axes created to visualize 3D poses."""

    alpha: float = 1.0  # Opacity value used for all colors (from 0 to 1)
    axis_length_m: float = 0.06  # Length (meters) of each x/y/z-axis
    axis_width_m: float = 0.005  # Width (m) of the axes

    def __post_init__(self) -> None:
        """Verify essential properties of the configured settings after they're initialized."""
        assert 0 < self.alpha <= 1, f"Alpha must be inside the interval (0,1]; was {self.alpha}."


class VisualizeWorldObjects:
    """A wrapper used to visualize a WorldObjects model of the environment."""

    def __init__(self, config: RGBAxesConfig) -> None:
        """Initialize the ROS publisher used to send marker messages to RViz.

        :param config: Settings for visual properties of the generated RViz markers
        """
        self.marker_pub = rospy.Publisher("visualization_marker", MarkerMsg, queue_size=10)
        self.config = config

        self._marker_id = 0  # Increments indefinitely

    def create_rgb_axes(self, pose: Pose3D) -> tuple[list[PointMsg], list[ColorRGBA]]:
        """Convert a Pose3D object into six points defining its RGB axes.

        We can conceptualize the pose as specifying pose_w_p: pose frame (p) w.r.t. world (w)

        :param pose: Pose in 3D space to be visualized using RGB axes
        :return: Tuple containing two lists of ROS messages: Six points in 3D and six RGBA colors.
        """
        axis_origin = Point3D(0, 0, 0)

        # Position of x/y/z-axis w.r.t. pose frame (p)
        position_p_x = Point3D(self.config.axis_length_m, 0, 0)
        position_p_y = Point3D(0, self.config.axis_length_m, 0)
        position_p_z = Point3D(0, 0, self.config.axis_length_m)

        x_axis_points = [pose @ axis_origin, pose @ position_p_x]
        y_axis_points = [pose @ axis_origin, pose @ position_p_y]
        z_axis_points = [pose @ axis_origin, pose @ position_p_z]
        all_points = x_axis_points + y_axis_points + z_axis_points
        point_msgs = [point_to_msg(p) for p in all_points]

        red_color = ColorRGBA(1, 0, 0, self.config.alpha)
        green_color = ColorRGBA(0, 1, 0, self.config.alpha)
        blue_color = ColorRGBA(0, 0, 1, self.config.alpha)
        color_msgs = [red_color, red_color, green_color, green_color, blue_color, blue_color]

        return point_msgs, color_msgs

    def create_marker_msg(self, obj_name: str, avg: AveragedPoseEstimate3D) -> MarkerMsg:
        """Convert the given averaged pose estimate into a visualization_msgs/Marker message.

        :param obj_name: Name of the object to which the given estimate corresponds
        :param avg: Pose estimate based on an average of many estimates
        :return: ROS message visualizing the pose estimates considered by the average
        """
        msg = MarkerMsg()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = DEFAULT_FRAME  # Visualize the poses w.r.t. the default frame

        msg.ns = obj_name
        msg.id = self._marker_id
        self._marker_id += 1
        msg.lifetime = rospy.Duration.from_sec(30)

        msg.type = MarkerMsg.LINE_LIST
        msg.action = MarkerMsg.ADD
        msg.scale.x = self.config.axis_width_m

        for pose_estimate in avg.all_estimates:
            pose_wrt_world = TransformManager.convert_to_frame(pose_estimate.pose, DEFAULT_FRAME)
            point_msgs, color_msgs = self.create_rgb_axes(pose_wrt_world)
            msg.points += point_msgs
            msg.colors += color_msgs

        return msg

    def visualize_world_objects(self, world_objects: WorldObjects[AveragedPoseEstimate3D]) -> None:
        """Visualize a collection of world object pose estimates by publishing markers to RViz.

        :param world_objects: Collection of object models at estimated poses
        """
        for obj_name, avg_pose_estimate in world_objects.pose_estimates.items():
            marker_msg = self.create_marker_msg(obj_name, avg_pose_estimate)
            self.marker_pub.publish(marker_msg)

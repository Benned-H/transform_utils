"""Record every object's first‑seen pose in the map frame to a YAML file."""

from __future__ import annotations
import threading, yaml, rospy, tf2_ros
from dataclasses import dataclass
from geometry_msgs.msg import TransformStamped
from pathlib import Path

@dataclass
class ObjectPoseRecorder:
    tag_system:      "AprilTagSystem"
    yaml_path:       Path
    map_frame:       str = "map"
    save_rate_hz:    float = 1.0          # how often we poll TF

    def __post_init__(self) -> None:
        self._buffer   = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer)
        self._known: dict[str, dict] = {}       # object‑name -> pose dict
        # load existing YAML (if any)
        if self.yaml_path.exists():
            with self.yaml_path.open() as f:
                data = yaml.safe_load(f) or {}
                self._known.update(data.get("objects", {}))

        # start background thread
        th = threading.Thread(target=self._loop, daemon=True)
        th.start()

    # ---------------- INTERNAL ----------------
    def _loop(self) -> None:
        rate = rospy.Rate(self.save_rate_hz)
        while not rospy.is_shutdown():
            # iterate over every object attached to every tag
            for tag in self.tag_system.tags.values():
                for obj_name in tag.relative_frames.keys():
                    if obj_name in self._known:      # already stored -> skip
                        continue
                    self._try_record(obj_name)
            rate.sleep()

    def _try_record(self, obj_frame: str) -> None:
        try:
            t: TransformStamped = self._buffer.lookup_transform(
                target_frame=self.map_frame,   # parent
                source_frame=obj_frame,        # child
                time=rospy.Time(0),            # latest
                timeout=rospy.Duration(0.3),
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return                            # transform not (yet) available

        pose_dict = {
            "position": {
                "x": t.transform.translation.x,
                "y": t.transform.translation.y,
                "z": t.transform.translation.z,
            },
            "orientation": {
                "x": t.transform.rotation.x,
                "y": t.transform.rotation.y,
                "z": t.transform.rotation.z,
                "w": t.transform.rotation.w,
            },
        }
        self._known[obj_frame] = pose_dict
        self._save_yaml()

    def _save_yaml(self) -> None:
        self.yaml_path.parent.mkdir(parents=True, exist_ok=True)
        with self.yaml_path.open("w") as f:
            yaml.safe_dump({"objects": self._known}, f, sort_keys=True)
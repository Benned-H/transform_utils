"""Define a class to convert timestamps between the local clock and a robot's clock."""

from __future__ import annotations

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum

from google.protobuf.timestamp_pb2 import Timestamp as TimestampProto
from rospy import Time as ROSTime

from transform_utils.logging import log_error

NSEC_PER_SEC = 10**9  # Nanoseconds per second


class SystemClock(Enum):
    """Specifies the system clock corresponding to a timestamp."""

    LOCAL = 1
    ROBOT = 2


@dataclass
class Timestamp:
    """A timestamp representing a specific time relative to the Unix epoch."""

    time_s: int  # Seconds since the Unix epoch began
    time_ns: int  # Nanoseconds since the timestamp's second began
    clock: SystemClock

    @classmethod
    def now(cls) -> Timestamp:
        """Construct a timestamp using the current time on the local machine.

        :return: Constructed Timestamp instance relative to the local clock
        """
        return Timestamp.from_time_s(time_s=time.time(), clock=SystemClock.LOCAL)

    @classmethod
    def from_time_s(cls, time_s: float, clock: SystemClock) -> Timestamp:
        """Construct a timestamp from a time (in seconds) since the Unix epoch.

        :param time_s: Time (seconds) since the epoch
        :param clock: System clock corresponding to the given time
        :return: Constructed Timestamp instance
        """
        rounded_time_s = int(time_s)
        remaining_time_s = time_s - rounded_time_s
        remaining_time_ns = int(remaining_time_s * NSEC_PER_SEC)

        return Timestamp(rounded_time_s, remaining_time_ns, clock)

    def to_time_s(self) -> tuple[float, SystemClock]:
        """Convert the timestamp into a clock time relative to the Unix epoch.

        :return: Tuple (time (sec) since Unix epoch started, relative clock)
        """
        time_s = self.time_s + self.time_ns / NSEC_PER_SEC
        return (time_s, self.clock)

    @classmethod
    def from_proto(cls, proto: TimestampProto) -> Timestamp:
        """Construct a Timestamp instance using the given Protobuf message.

        :param proto: Protobuf message (assumed to be relative to the robot clock)
        :return: Constructed Timestamp instance
        """
        return Timestamp(time_s=proto.seconds, time_ns=proto.nanos, clock=SystemClock.ROBOT)

    @classmethod
    def from_ros_time(cls, ros_time: ROSTime) -> Timestamp:
        """Construct a Timestamp instance from the given rospy.Time instance.

        :param ros_time: ROS timestamp representation (assumed to be relative to the local clock)
        :return: Constructed Timestamp instance
        """
        return Timestamp(time_s=ros_time.secs, time_ns=ros_time.nsecs, clock=SystemClock.LOCAL)

    def add_offset_s(self, offset_s: float) -> Timestamp:
        """Construct a timestamp by adding the given duration (sec) to this timestamp.

        :param offset_s: Duration (seconds) the constructed timestamp is offset from this one
        :return: Constructed Timestamp instance
        """
        time_s, clock = self.to_time_s()
        return Timestamp.from_time_s(time_s + offset_s, clock)


@dataclass
class SyncResult:
    """A struct organizing information obtained during a time synchronization."""

    robot_clock_skew_s: float  # Estimated robot clock skew from the local clock (seconds)
    round_trip_time_s: float  # Current duration (seconds) of a request-response trip


class TimeSync(ABC):
    """A wrapper to manage converting between the local clock and robot clock."""

    def __init__(self) -> None:
        """Initialize member variables to their initial values."""
        self.latest_sync_result: SyncResult | None = None  # Result from latest time-sync

        # Maximum duration (seconds) of any request-response round trip
        self.max_round_trip_s: float | None = None

        self.max_sync_duration_s: float | None = None  # Max. duration (sec) a time-sync has taken

        self.total_sync_time_s = 0.0  # Total time (seconds) spent synchronizing with the robot
        self.total_sync_count = 0  # Number of completed calls to resync()

        self.resync()

    @abstractmethod
    def _sync(self) -> SyncResult | None:
        """Synchronize the TimeSync with the robot's clock.

        :return: Latest data from robot time-sync (None if synchronization fails)
        """

    def resync(self) -> None:
        """Resynchronize the local clock with the robot clock."""
        start_time_s = time.time()
        self.latest_sync_result = self._sync()
        end_time_s = time.time()

        sync_duration_s = end_time_s - start_time_s
        if self.max_sync_duration_s is None:
            self.max_sync_duration_s = sync_duration_s
        else:
            self.max_sync_duration_s = max(self.max_sync_duration_s, sync_duration_s)

        if self.max_round_trip_s is None:
            self.max_round_trip_s = self.latest_sync_result.round_trip_time_s
        else:
            self.max_round_trip_s = max(
                self.max_round_trip_s,
                self.latest_sync_result.round_trip_time_s,
            )

        self.total_sync_time_s += sync_duration_s
        self.total_sync_count += 1

    @property
    def average_sync_duration_s(self) -> float | None:
        """Calculate the average duration (in seconds) of each time-sync so far.

        :return: Average duration (seconds) of calls to resync(), else None if no calls yet
        """
        if self.total_sync_time_s == 0.0 or self.total_sync_count == 0:
            return None
        return self.total_sync_time_s / self.total_sync_count

    def change_relative_clock(self, ts: Timestamp, rel_clock: SystemClock) -> Timestamp | None:
        """Convert the given timestamp to be relative to the specified system clock.

        :param ts: Timestamp to be converted
        :param rel_clock: System clock corresponding to the resulting timestamp
        :return: Converted timestamp w.r.t. the requested clock, or None if clock skew is unknown
        """
        if ts.clock == rel_clock:
            return ts

        if self.latest_sync_result is None:
            log_error("Cannot convert relative clock; robot clock skew is unknown.")
            return None

        time_s, ts_clock = ts.to_time_s()

        if ts_clock == SystemClock.LOCAL and rel_clock == SystemClock.ROBOT:
            robot_time_s = time_s + self.latest_sync_result.robot_clock_skew_s
            return Timestamp.from_time_s(robot_time_s, SystemClock.ROBOT)

        if ts_clock == SystemClock.ROBOT and rel_clock == SystemClock.LOCAL:
            local_time_s = time_s - self.latest_sync_result.robot_clock_skew_s
            return Timestamp.from_time_s(local_time_s, SystemClock.LOCAL)

        log_error(f"Unable to convert timestamp relative to {ts_clock} to clock {rel_clock}.")
        return None

    def convert_to_robot_proto(self, ts: Timestamp | ROSTime) -> TimestampProto | None:
        """Convert the given timestamp into a Protobuf message relative to the robot clock.

        :param ts: Timestamp (or rospy.Time) specifying a time relative to the Unix epoch
        :return: Constructed Protobuf message, or None if conversion fails
        """
        if isinstance(ts, ROSTime):
            ts = Timestamp.from_ros_time(ts)

        robot_ts = self.change_relative_clock(ts, SystemClock.ROBOT)
        if robot_ts is None:
            return None

        return TimestampProto(seconds=robot_ts.time_s, nanos=robot_ts.time_ns)

    def convert_to_ros_time(self, ts: Timestamp | TimestampProto) -> ROSTime | None:
        """Convert the given timestamp into a rospy.Time primitive relative to the local clock.

        :param ts: Timestamp (or Protobuf) specifying a time relative to the Unix epoch
        :return: Constructed rospy.Time instance, or None if conversion fails
        """
        if isinstance(ts, TimestampProto):
            ts = Timestamp.from_proto(ts)

        local_ts = self.change_relative_clock(ts, rel_clock=SystemClock.LOCAL)
        local_time_s, _ = local_ts.to_time_s()
        return None if local_ts is None else ROSTime.from_seconds(local_time_s)

    def get_future_ref_time_proto(self) -> TimestampProto:
        """Create a Protobuf message marking a future reference time for a robot command.

        :return: Constructed Timestamp Protobuf message
        """
        future_proof_s = 2.0 * self.max_round_trip_s + 0.1
        ref_time_local_ts = Timestamp.now().add_offset_s(future_proof_s)

        # Offset by the opposite of the robot clock skew (i.e., "toward local" one extra skew)
        #   so that when the Spot SDK does so with the reference time, things are correct
        adjusted_ts = ref_time_local_ts.add_offset_s(-self.latest_sync_result.robot_clock_skew_s)
        return self.convert_to_robot_proto(adjusted_ts)

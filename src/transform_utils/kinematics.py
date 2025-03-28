"""Define dataclasses to represent 3D positions, orientations, and transforms."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict

import numpy as np
from tf.transformations import (
    euler_from_quaternion,
    quaternion_from_euler,
    quaternion_from_matrix,
    quaternion_matrix,
)

Configuration = Dict[str, float]  # Map from joint names to positions (rad or m)


@dataclass
class Point3D:
    """An (x,y,z) position in 3D space."""

    x: float
    y: float
    z: float

    @classmethod
    def identity(cls) -> Point3D:
        """Construct a Point3D corresponding to the identity translation."""
        return Point3D(0, 0, 0)

    def to_array(self) -> np.ndarray:
        """Convert the point to a NumPy array."""
        return np.array([self.x, self.y, self.z])

    @classmethod
    def from_array(cls, arr: np.ndarray) -> Point3D:
        """Construct a Point3D from the given NumPy array."""
        assert arr.shape == (3,), "3D position must be a three-element vector."
        return cls(arr[0], arr[1], arr[2])

    def approx_equal(self, other: Point3D) -> bool:
        """Check whether another point is approximately equal to this point."""
        return np.allclose(self.to_array(), other.to_array())


@dataclass
class Quaternion:
    """A unit quaternion representing a 3D orientation."""

    x: float
    y: float
    z: float
    w: float

    def __post_init__(self) -> None:
        """Normalize the quaternion after it is initialized."""
        self.normalize()

    def normalize(self) -> None:
        """Normalize the quaternion to ensure that it is a unit quaternion."""
        norm = float(np.linalg.norm(self.to_array()))
        assert norm > 0, f"Cannot normalize near-zero quaternion: {self}"

        self.x /= norm
        self.y /= norm
        self.z /= norm
        self.w /= norm

    @classmethod
    def identity(cls) -> Quaternion:
        """Construct a quaternion corresponding to the identity rotation."""
        return cls(0, 0, 0, 1)

    def to_array(self) -> np.ndarray:
        """Convert the quaternion to a NumPy array."""
        return np.array([self.x, self.y, self.z, self.w])

    @classmethod
    def from_array(cls, arr: np.ndarray) -> Quaternion:
        """Construct a quaternion from a NumPy array."""
        assert arr.shape == (4,), "Quaternion must be a four-element vector."
        return cls(arr[0], arr[1], arr[2], arr[3])

    def to_euler_rpy(self) -> tuple[float, float, float]:
        """Convert the quaternion to Euler roll, pitch, and yaw angles.

        :return: Tuple of (roll, pitch, yaw) angles in radians
        """
        r, p, y = euler_from_quaternion(self.to_array())
        return (r, p, y)

    @classmethod
    def from_euler_rpy(cls, roll_rad: float, pitch_rad: float, yaw_rad: float) -> Quaternion:
        """Construct a quaternion from three fixed-frame Euler angles.

        Note: By default, the quaternion_from_euler() function uses: axes="sxyz"
            meaning roll, then pitch, then yaw, all in a static (i.e., fixed) frame.

        :param roll_rad: Roll angle about the x-axis (radians)
        :param pitch_rad: Pitch angle about the y-axis (radians)
        :param yaw_rad: Yaw angle about the z-axis (radians)
        :return: Unit quaternion corresponding to the Euler angles
        """
        return cls.from_array(quaternion_from_euler(roll_rad, pitch_rad, yaw_rad))

    def to_rotation_matrix(self) -> np.ndarray:
        """Convert the quaternion to a 3x3 rotation matrix."""
        matrix = quaternion_matrix([self.x, self.y, self.z, self.w])
        assert matrix.shape == (4, 4), f"Homogeneous matrix must be 4x4; was {matrix.shape}."
        return matrix[:3, :3]

    @classmethod
    def from_rotation_matrix(cls, r_matrix: np.ndarray) -> Quaternion:
        """Construct a quaternion from a 3x3 rotation matrix."""
        assert r_matrix.shape == (3, 3), f"Expected a 3x3 matrix; received {r_matrix.shape}."
        homogeneous_r_matrix = np.eye(4)
        homogeneous_r_matrix[:3, :3] = r_matrix
        q_array = quaternion_from_matrix(homogeneous_r_matrix)
        return cls.from_array(q_array)


DEFAULT_FRAME = "map"


@dataclass
class Pose2D:
    """A position and orientation on the 2D plane."""

    x: float
    y: float
    yaw_rad: float
    ref_frame: str = DEFAULT_FRAME  # Frame of reference for the pose

    @classmethod
    def from_list(cls, x_y_yaw: list[float], ref_frame: str = DEFAULT_FRAME) -> Pose2D:
        """Construct a Pose2D from the given list of X-Y-Yaw data.

        :param x_y_yaw: List of three floats specifying (x,y,yaw)
        :param ref_frame: Reference frame of the constructed Pose2D
        :return: Pose2D constructed using the given values
        """
        assert len(x_y_yaw) == 3, f"Cannot construct Pose2D from list of length {len(x_y_yaw)}."
        x, y, yaw_rad = x_y_yaw
        return Pose2D(x, y, yaw_rad, ref_frame)


@dataclass
class Pose3D:
    """A position and orientation in 3D space."""

    position: Point3D
    orientation: Quaternion
    ref_frame: str = DEFAULT_FRAME  # Frame of reference for the pose

    @property
    def yaw_rad(self) -> float:
        """Retrieve the Pose3D's yaw about the z-axis (in radians)."""
        _, _, yaw_rad = self.orientation.to_euler_rpy()
        return yaw_rad

    @classmethod
    def identity(cls) -> Pose3D:
        """Construct a Pose3D corresponding to the identity transformation."""
        return Pose3D(Point3D.identity(), Quaternion.identity())

    def __matmul__(self, other: Pose3D) -> Pose3D:
        """Multiply the homogeneous transformation matrices of this pose and another pose."""
        m1 = self.to_homogeneous_matrix()
        m2 = other.to_homogeneous_matrix()
        result_ref_frame = self.ref_frame  # Result takes the "leftmost" reference frame
        return Pose3D.from_homogeneous_matrix(m1 @ m2, result_ref_frame)

    def inverse(self, ref_frame: str) -> Pose3D:
        """Compute the pose corresponding to the inverse transformation of this pose.

        :param ref_frame: Reference frame used for the resulting pose
        :return: Inverse transformation of this pose w.r.t. the reference frame
        """
        inv_transform = np.linalg.inv(self.to_homogeneous_matrix())
        return Pose3D.from_homogeneous_matrix(inv_transform, ref_frame)

    @classmethod
    def from_2d(cls, pose: Pose2D) -> Pose3D:
        """Construct a Pose3D from a Pose2D.

        :param pose: Pose in 2D space
        :return: 3D pose corresponding to the given 2D pose
        """
        return Pose3D.from_xyz_rpy(
            x=pose.x,
            y=pose.y,
            yaw_rad=pose.yaw_rad,
            ref_frame=pose.ref_frame,
        )

    def to_2d(self) -> Pose2D:
        """Convert the pose into a 2D pose, discarding any 3D-only information."""
        return Pose2D(self.position.x, self.position.y, self.yaw_rad, self.ref_frame)

    @classmethod
    def from_xyz_rpy(
        cls,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        roll_rad: float = 0.0,
        pitch_rad: float = 0.0,
        yaw_rad: float = 0.0,
        ref_frame: str = DEFAULT_FRAME,
    ) -> Pose3D:
        """Construct a Pose3D from the given XYZ coordinates and RPY angles.

        :param x: Translation's x-coordinate
        :param y: Translation's y-coordinate
        :param z: Translation's z-coordinate
        :param roll_rad: Fixed-frame roll angle (radians) about the x-axis
        :param pitch_rad: Fixed-frame pitch angle (radians) about the y-axis
        :param yaw_rad: Fixed-frame yaw angle (radians) about the z-axis
        :param ref_frame: Reference frame of the constructed pose

        :return: Pose3D constructed using the given values
        """
        position = Point3D(x, y, z)
        orientation = Quaternion.from_euler_rpy(roll_rad, pitch_rad, yaw_rad)

        return cls(position, orientation, ref_frame)

    @classmethod
    def from_list(cls, xyz_rpy: list[float], ref_frame: str = DEFAULT_FRAME) -> Pose3D:
        """Construct a Pose3D from the given list of XYZ-RPY data.

        :param xyz_rpy: List of six floats specifying (x,y,z,roll,pitch,yaw)
        :param ref_frame: Reference frame of the constructed Pose3D
        :return: Pose3D constructed using the given values
        """
        assert len(xyz_rpy) == 6, f"Cannot construct Pose3D from list of length {len(xyz_rpy)}."
        x, y, z, roll, pitch, yaw = xyz_rpy
        return Pose3D.from_xyz_rpy(x, y, z, roll, pitch, yaw, ref_frame)

    def to_xyz_rpy(self) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        """Convert the pose into the corresponding (x, y, z) and (roll, pitch, yaw) tuples.

        :return: Pair of tuples (x, y, z) and (roll, pitch, yaw) with angles in radians
        """
        xyz = (self.position.x, self.position.y, self.position.z)
        rpy = self.orientation.to_euler_rpy()
        return (xyz, rpy)

    @classmethod
    def from_homogeneous_matrix(cls, matrix: np.ndarray, ref_frame: str = DEFAULT_FRAME) -> Pose3D:
        """Construct a Pose3D from a 4x4 homogeneous transformation matrix."""
        assert matrix.shape == (4, 4), f"Expected a 4x4 matrix; received {matrix.shape}."
        position = Point3D(matrix[0, 3], matrix[1, 3], matrix[2, 3])
        orientation = Quaternion.from_rotation_matrix(matrix[:3, :3])
        return cls(position, orientation, ref_frame)

    def to_homogeneous_matrix(self) -> np.ndarray:
        """Convert the Pose3D to a 4x4 homogeneous transformation matrix."""
        matrix = np.eye(4)
        matrix[:3, :3] = self.orientation.to_rotation_matrix()
        matrix[:3, 3] = [self.position.x, self.position.y, self.position.z]
        return matrix

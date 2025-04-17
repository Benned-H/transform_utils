"""Define unit tests for the Pose3D class."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest

from transform_utils.filesystem.load_from_yaml import load_yaml_into_dict
from transform_utils.kinematics import DEFAULT_FRAME, Pose3D


@pytest.fixture
def poses_yaml_data() -> dict[str, Any]:
    """Return pose data imported from a known-good YAML file stored under `tests/data`."""
    yaml_path = Path(__file__).parent / "data" / "pose_examples.yaml"
    assert yaml_path.exists(), f"Cannot find the YAML file: {yaml_path}"

    yaml_data: dict[str, Any] = load_yaml_into_dict(yaml_path)
    assert "poses" in yaml_data, f"Expected key 'poses' in YAML file: {yaml_path}."

    poses_data: dict[str, Any] = yaml_data["poses"]
    return poses_data


def test_pose3d_from_yaml(poses_yaml_data: dict[str, Any]) -> None:
    """Verify that Pose3D instances can be imported from a known-good example YAML file."""
    # Arrange: The YAML data is provided via Pytest fixture
    expected_pose_a = Pose3D.from_list([3, 2, 1, 0, 0.7854, 0], ref_frame=DEFAULT_FRAME)
    expected_pose_b = Pose3D.from_list([0, 2, 1, 0, 0, 0], ref_frame=DEFAULT_FRAME)
    expected_pose_c = Pose3D.from_list([4, 5, 6, 0, 0, 0], ref_frame="frame_c")

    expected_poses: dict[str, Pose3D] = {
        "pose_a": expected_pose_a,
        "pose_b": expected_pose_b,
        "pose_c": expected_pose_c,
    }

    # Act: Convert each pose from YAML data into a Pose3D instance
    loaded_poses: dict[str, Pose3D] = {}
    for pose_name, pose_data in poses_yaml_data.items():
        loaded_poses[pose_name] = Pose3D.from_yaml(pose_data, default_frame=DEFAULT_FRAME)

    # Assert: Verify that the loaded Pose3D instances match the expected values
    for pose_name, expected_pose in expected_poses.items():
        result_pose = loaded_poses.get(pose_name)
        assert result_pose is not None, f"Expected a pose named '{pose_name}'."

        assert result_pose.approx_equal(expected_pose)
        assert result_pose.ref_frame == expected_pose.ref_frame

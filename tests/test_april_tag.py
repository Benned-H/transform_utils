"""Define unit tests for the AprilTag and AprilTagSystem dataclasses."""

from pathlib import Path

import pytest

from transform_utils.kinematics import Pose3D
from transform_utils.perception.april_tag import AprilTagSystem


@pytest.fixture
def apriltag_yaml_path() -> Path:
    """Return the path to a known-good YAML file of AprilTag data stored under `tests/data`."""
    yaml_path = Path(__file__).parent / "data" / "apriltags_example.yaml"
    assert yaml_path.exists()
    return yaml_path


def test_april_tag_system_from_yaml(apriltag_yaml_path: Path) -> None:
    """Verify that the AprilTagSystem class can be imported from YAML."""
    # Arrange: The YAML file path is provided via Pytest fixture
    expected_expo_spray_pose = Pose3D.from_list([0.1, 0, 0, 0, 0, 0], ref_frame="tag_60")
    expected_table_pose = Pose3D.from_list([-0.1, 0, -0.5, 1.5708, 0, 0], ref_frame="tag_51")
    expected_cabinet_pose = Pose3D.from_list([0.32, 0.24, 0.18, 0, 0, 0], ref_frame="tag_1")

    # Act: Import the AprilTag system data from the example YAML file
    system_result = AprilTagSystem.from_yaml(apriltag_yaml_path)

    # Assert: Verify that the loaded AprilTags data matches the expected values
    tag_1 = system_result.tags.get(1)
    tag_51 = system_result.tags.get(51)
    tag_60 = system_result.tags.get(60)

    assert tag_1 is not None, "Expected an AprilTag with the ID 1."
    assert tag_51 is not None, "Expected an AprilTag with the ID 51."
    assert tag_60 is not None, "Expected an AprilTag with the ID 60."

    assert tag_1.size_cm == pytest.approx(7.62)
    assert "cabinet_handle" in tag_1.relative_frames
    assert tag_1.in_bundle, "Expected tag 1 to be part of a bundle of tags."

    result_cabinet_pose = tag_1.relative_frames["cabinet_handle"]
    assert result_cabinet_pose.approx_equal(expected_cabinet_pose)

    assert tag_51.size_cm == pytest.approx(18.5)
    assert "table" in tag_51.relative_frames
    assert not tag_51.in_bundle, "Expected tag 51 to be a single tag."

    result_table_pose = tag_51.relative_frames["table"]
    assert result_table_pose.approx_equal(expected_table_pose)

    assert tag_60.size_cm == pytest.approx(4.0)
    assert "expo_spray" in tag_60.relative_frames
    assert not tag_60.in_bundle, "Expected tag 60 to be a single tag."

    result_expo_spray_pose = tag_60.relative_frames["expo_spray"]
    assert result_expo_spray_pose.approx_equal(expected_expo_spray_pose)

    # Verify that the imported cameras can detect the correct AprilTag IDs
    hand_detects = system_result.camera_detects_tags.get("hand")
    frontleft_detects = system_result.camera_detects_tags.get("frontleft")
    frontright_detects = system_result.camera_detects_tags.get("frontright")

    assert hand_detects is not None, "Expected a camera named 'hand'."
    assert frontleft_detects is not None, "Expected a camera named 'frontleft'."
    assert frontright_detects is not None, "Expected a camera named 'frontright'."

    assert hand_detects == {1, 60}
    assert frontleft_detects == {1, 51}
    assert frontright_detects == {1, 51}

    assert system_result.camera_topic_prefix == "/ar_pose_marker"

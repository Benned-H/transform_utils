"""Define unit tests for the AprilTag and AprilTagSystem dataclasses."""

from pathlib import Path

import pytest

from transform_utils.kinematics import Pose3D
from transform_utils.world_model.april_tag import AprilTagSystem


@pytest.fixture
def apriltag_yaml_path() -> Path:
    """Return the path to a known-good YAML file of AprilTag data stored under `tests/data`."""
    yaml_path = Path(__file__).parent / "data" / "apriltags_example.yaml"
    assert yaml_path.exists()
    return yaml_path


def test_april_tag_system_from_yaml(apriltag_yaml_path: Path) -> None:
    """Verify that the AprilTagSystem class can be imported from YAML."""
    # Arrange: The YAML file path is provided via Pytest fixture
    expected_expo_spray_pose = Pose3D.from_list([1, 2, 3, 0, 0, 0], ref_frame="tag_1")
    expected_table_pose = Pose3D.from_list([4, 5, 6, 0, 0, 0], ref_frame="tag_101")

    # Act: Import the AprilTag system data from the example YAML file
    system_result = AprilTagSystem.from_yaml(apriltag_yaml_path)

    # Assert: Verify that the loaded AprilTags data matches the expected values
    tag_1 = system_result.tags.get(1)
    tag_101 = system_result.tags.get(101)

    assert tag_1 is not None, "Expected an AprilTag with the ID 1."
    assert tag_101 is not None, "Expected an AprilTag with the ID 101."

    assert tag_1.size_cm == pytest.approx(3.3)
    assert "expo_spray" in tag_1.relative_frames

    result_expo_spray_pose = tag_1.relative_frames["expo_spray"]
    assert result_expo_spray_pose.approx_equal(expected_expo_spray_pose)

    assert tag_101.size_cm == pytest.approx(4.4)
    assert "table" in tag_101.relative_frames

    result_table_pose = tag_101.relative_frames["table"]
    assert result_table_pose.approx_equal(expected_table_pose)

    # Verify that the imported cameras can detect the correct AprilTag IDs
    hand_detects = system_result.camera_detects_tags.get("hand")
    frontleft_detects = system_result.camera_detects_tags.get("frontleft")
    right_detects = system_result.camera_detects_tags.get("right")

    assert hand_detects is not None, "Expected a camera named 'hand'."
    assert frontleft_detects is not None, "Expected a camera named 'frontleft'."
    assert right_detects is not None, "Expected a camera named 'right'."

    assert hand_detects == {1}
    assert frontleft_detects == {101}
    assert right_detects == {1, 101}

    assert system_result.camera_topic_prefix == "/ar_track_alvar/camera/"

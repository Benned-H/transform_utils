"""Define unit tests for the Landmarks2D class."""

from pathlib import Path

import pytest

from transform_utils.states.landmarks_2d import Landmarks2D


@pytest.fixture
def landmarks_yaml_path() -> Path:
    """Return the path to a known-good YAML file stored under `tests/yaml_examples`."""
    return Path(__file__).parent / "yaml_examples" / "landmarks_2d_example.yaml"


def test_landmarks_2d_from_example_yaml(landmarks_yaml_path: Path) -> None:
    """Verify that a Landmarks2D instance can be imported from a known-good example YAML."""
    # Arrange: The path to the YAML file is provided via Pytest fixture

    # Act: Create a Landmarks2D instance using the example YAML file
    landmarks_result = Landmarks2D.from_yaml(landmarks_yaml_path)

    # Assert: Verify that the loaded landmarks match the expected values
    pose_a = landmarks_result.get("landmark_a")
    assert pose_a is not None, "Expected landmark_a to be loaded from YAML."
    assert pose_a.x == pytest.approx(1)
    assert pose_a.y == pytest.approx(2)
    assert pose_a.yaw_rad == pytest.approx(3.14)
    assert pose_a.ref_frame == "default_frame"

    pose_b = landmarks_result.get("landmark_b")
    assert pose_b is not None, "Expected landmark_b to be loaded from YAML."
    assert pose_b.x == pytest.approx(3)
    assert pose_b.y == pytest.approx(4)
    assert pose_b.yaw_rad == pytest.approx(1.57)
    assert pose_b.ref_frame == "custom_frame"

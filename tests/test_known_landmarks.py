"""Define unit tests for the KnownLandmarks2D class."""

from pathlib import Path

import pytest

from transform_utils.world_model.known_landmarks import KnownLandmarks2D


@pytest.fixture
def known_landmarks_yaml_path() -> Path:
    """Return the path to a known-good YAML file stored under `tests/data`."""
    return Path(__file__).parent / "data" / "known_landmarks_example.yaml"


def test_known_landmarks_from_example_yaml(known_landmarks_yaml_path: Path) -> None:
    """Verify that a KnownLandmarks2D instance can be imported from a known-good example YAML.

    :param known_landmarks_yaml_path: Path to a known-good landmarks YAML file
    """
    # Arrange: The path to the YAML file is provided via Pytest fixture

    # Act: Create a KnownLandmarks2D instance using the example YAML file
    landmarks_result = KnownLandmarks2D.from_yaml(known_landmarks_yaml_path)

    # Assert: Verify that the loaded landmarks match the expected values
    pose_a = landmarks_result.landmarks.get("landmark_a")
    assert pose_a is not None, "Expected landmark_a to be loaded from YAML."
    assert pose_a.x == pytest.approx(1)
    assert pose_a.y == pytest.approx(2)
    assert pose_a.yaw_rad == pytest.approx(0)
    assert pose_a.ref_frame == "default_frame"

    pose_b = landmarks_result.landmarks.get("landmark_b")
    assert pose_b is not None, "Expected landmark_b to be loaded from YAML."
    assert pose_b.x == pytest.approx(3)
    assert pose_b.y == pytest.approx(4)
    assert pose_b.yaw_rad == pytest.approx(1.57)
    assert pose_b.ref_frame == "custom_frame"

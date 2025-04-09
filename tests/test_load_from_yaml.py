"""Define unit tests for utility functions for loading data from YAML."""

import tempfile
from pathlib import Path

import pytest

from transform_utils.filesystem.load_from_yaml import (
    load_apriltag_relative_transforms_from_yaml,
    load_yaml_into_dict,
)


def test_load_yaml_into_dict_file_not_found() -> None:
    """Verify that load_yaml_into_dict returns an empty dictionary when the file is missing."""
    # Arrange: Create a temporary directory and define a non-existent YAML file
    with tempfile.TemporaryDirectory() as temp_dir:
        missing_file = Path(temp_dir) / "nonexistent.yaml"

        # Act: Call load_yaml_into_dict on the missing file
        result = load_yaml_into_dict(missing_file)

        # Assert: The result should be an empty dictionary
        assert result == {}


@pytest.fixture
def apriltag_yaml_path() -> Path:
    """Return the path to a known-good YAML file of AprilTag data stored under `tests/data`."""
    yaml_path = Path(__file__).parent / "data" / "apriltag_transforms_example.yaml"
    assert yaml_path.exists()
    return yaml_path


def test_load_apriltag_relative_transforms(apriltag_yaml_path: Path) -> None:
    """Verify that AprilTag-relative object transforms can be imported from YAML."""
    # Arrange: The YAML file path is provided via Pytest fixture

    # Act: Import the AprilTag data from the example YAML file
    transforms_result = load_apriltag_relative_transforms_from_yaml(apriltag_yaml_path)

    # Assert: Verify that the loaded relative transforms match the expected values
    expo_spray_pose = transforms_result.get("expo_spray")
    table_pose = transforms_result.get("table1")

    assert expo_spray_pose is not None, "Expected a relative pose for 'expo_spray'."
    assert table_pose is not None, "Expected a relative pose for 'table1'."

    assert expo_spray_pose.ref_frame == "tag-39"
    assert expo_spray_pose.position.x == 1
    assert expo_spray_pose.position.y == 2
    assert expo_spray_pose.position.z == 3

    assert table_pose.ref_frame == "tag-41"
    assert table_pose.position.x == 4
    assert table_pose.position.y == 5
    assert table_pose.position.z == 6

"""Define unit tests for the KinematicTree class."""

from pathlib import Path

import pytest

from transform_utils.states.kinematic_tree import KinematicTree


@pytest.fixture
def environment_yaml_path() -> Path:
    """Return the path to a known-good YAML file stored under `tests/yaml_examples`."""
    return Path(__file__).parent / "yaml_examples" / "environment_example.yaml"


def test_kinematic_tree_from_yaml(environment_yaml_path: Path) -> None:
    """Verify that a KinematicTree is correctly imported from YAML."""
    # Act: Create a KinematicTree instance using the example YAML file
    tree = KinematicTree.from_yaml(environment_yaml_path)

    # Assert: Verify that the loaded data matches the expected values
    spot_pose = tree.robot_base_poses.get("spot")
    assert spot_pose is not None, "Robot base pose for 'spot' not loaded."
    assert spot_pose.x == pytest.approx(3.0)
    assert spot_pose.y == pytest.approx(4.0)
    assert spot_pose.z == pytest.approx(0.5)
    assert spot_pose.yaw_rad == pytest.approx(-1.5708)

    # Verify that the eraser object's imported pose is correct
    eraser_pose = tree.object_poses.get("eraser1")
    assert eraser_pose is not None, "Pose for 'eraser1' not loaded."
    assert eraser_pose.x == pytest.approx(2.0)
    assert eraser_pose.y == pytest.approx(3.0)
    assert eraser_pose.z == pytest.approx(0.5)
    assert eraser_pose.ref_frame == "map"

    # Verify that the 'door_to_lab' landmark is correct
    door_pose = tree.landmarks.get("door_to_lab")
    assert door_pose is not None, "Landmark 'door_to_lab' not loaded."
    assert door_pose.x == pytest.approx(4.0)
    assert door_pose.y == pytest.approx(5.0)
    assert door_pose.yaw_rad == pytest.approx(3.1416)
    assert door_pose.ref_frame == "map"

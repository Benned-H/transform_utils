"""Define unit tests for the WorldModel class."""

from pathlib import Path

import pytest

from transform_utils.world_model.world_model import WorldModel


@pytest.fixture
def world_model_yaml_path() -> Path:
    """Return the path to a known-good YAML file stored under `tests/data`."""
    return Path(__file__).parent / "data" / "world_model_example.yaml"


def test_world_model_from_yaml(world_model_yaml_path: Path) -> None:
    """Verify that a WorldModel is correctly imported from YAML.

    :param world_model_yaml_path: Path to a known-good world model YAML file
    """
    # Act: Create a WorldModel instance using the example YAML file
    world = WorldModel.from_yaml(world_model_yaml_path)

    # Assert: Verify that the loaded world matches the expected values
    spot_pose = world.kinematic_state.robot_base_poses.get("spot")
    assert spot_pose is not None, "Robot base pose for 'spot' not loaded."
    assert spot_pose.x == pytest.approx(3.0)
    assert spot_pose.y == pytest.approx(4.0)
    assert spot_pose.z == pytest.approx(0.5)
    assert spot_pose.yaw_rad == pytest.approx(-1.5708)

    # Verify that the eraser object model is correct
    assert "eraser1" in world.object_names, "Object model 'eraser1' is missing."
    assert world.object_models["eraser1"].name == "eraser1"
    assert world.object_models["eraser1"].object_type == "eraser"

    # Verify that the eraser's imported pose is correct
    eraser_pose = world.kinematic_state.object_poses.get("eraser1")
    assert eraser_pose is not None, "Pose for 'eraser1' not loaded."
    assert eraser_pose.x == pytest.approx(2.0)
    assert eraser_pose.y == pytest.approx(3.0)
    assert eraser_pose.z == pytest.approx(0.5)
    assert eraser_pose.ref_frame == "map"

    # Verify that the 'door_to_lab' landmark is correct
    door_pose = world.landmarks.landmarks.get("door_to_lab")
    assert door_pose is not None, "Landmark 'door_to_lab' not loaded."
    assert door_pose.x == pytest.approx(4.0)
    assert door_pose.y == pytest.approx(5.0)

    # --- Landmarks ------------------------------------------------------------
    door_pose = world.landmarks.landmarks.get("door_to_lab")
    assert door_pose is not None, "Landmark 'door_to_lab' not loaded."
    assert door_pose.x == pytest.approx(4.0)
    assert door_pose.y == pytest.approx(5.0)
    assert door_pose.yaw_rad == pytest.approx(3.1416)
    assert door_pose.ref_frame == "map"

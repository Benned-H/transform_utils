"""Define unit tests for utility functions for loading data from YAML."""

import tempfile
from pathlib import Path

from transform_utils.world_model.load_from_yaml import load_yaml_into_dict


def test_load_yaml_into_dict_file_not_found() -> None:
    """Verify that load_yaml_into_dict returns an empty dictionary when the file is missing."""
    # Arrange: Create a temporary directory and define a non-existent YAML file
    with tempfile.TemporaryDirectory() as temp_dir:
        missing_file = Path(temp_dir) / "nonexistent.yaml"

        # Act: Call load_yaml_into_dict on the missing file
        result = load_yaml_into_dict(missing_file)

        # Assert: The result should be an empty dictionary
        assert result == {}

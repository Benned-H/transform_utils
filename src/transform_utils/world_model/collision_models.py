"""Define dataclasses to represent collision models for objects in the environment."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Tuple

import numpy as np
import trimesh

from transform_utils.logging import log_info
from transform_utils.ros.ros_utils import resolve_package_path

ObjectDims = Tuple[float, float, float]  # Represents object-frame size in (x,y,z)


@dataclass
class CollisionMesh:
    """An object's collision mesh, imported from file."""

    mesh: trimesh.Trimesh
    path: Path  # Path to the object's collision mesh file
    import_steps: list[str]  # Sequence of operations to normalize the mesh (can be empty)
    dimensions: ObjectDims  # Mesh bounding box size in (x, y, z)

    @classmethod
    def from_yaml(cls, mesh_data: dict[str, Any]) -> CollisionMesh:
        """Construct a CollisionMesh instance from a dictionary of YAML data.

        :param mesh_data: Dictionary of collision mesh data imported from YAML
        :return: Constructed CollisionMesh instance
        :raises: FileNotFoundError, if the YAML-specified mesh filepath is invalid
        """
        relative_path = mesh_data["filepath"]  # Package-relative path
        absolute_path = resolve_package_path(relative_path)

        if absolute_path is None or not absolute_path.exists():
            error = f"Unable to load a mesh from the path: {relative_path}."
            raise FileNotFoundError(error)

        steps = mesh_data.get("steps", [])

        mesh = load_trimesh(absolute_path, steps)
        min_bounds, max_bounds = mesh.bounds
        dims = tuple((max_bounds - min_bounds).tolist())

        return CollisionMesh(
            mesh=mesh,
            path=absolute_path,
            import_steps=steps,
            dimensions=dims,
        )


def load_trimesh(mesh_path: Path, import_steps: list[str | dict]) -> trimesh.Trimesh:
    """Load an object's mesh from file.

    During import, the operations prescribed by "import_steps" are applied to the mesh.

    :param mesh_path: Path to the mesh file to be imported
    :param import_steps: Sequence of geometric operations applied to the mesh (e.g., "center_xy")
    :return: Normalized mesh imported using the trimesh library
    """
    mesh = trimesh.load(mesh_path, force="mesh")
    if not isinstance(mesh, trimesh.Trimesh):
        error = f"{mesh_path} imported as unexpected type: {type(mesh)}"
        raise TypeError(error)

    # mesh = mesh.convert_units("meters", guess=False)

    for step in import_steps:
        (min_x, min_y, min_z), (max_x, max_y, max_z) = mesh.bounds

        if step == "orient_bb":  # Axis-align the mesh using its oriented bounding box
            mesh.apply_obb()
        elif step == "bottom":  # Translate the mesh so that z = 0 aligns with its bottom
            mesh.apply_translation(np.array([0, 0, -min_z]))
        elif step == "center_xy":  # Translate the mesh so that its center has (x,y) = 0
            x_size = max_x - min_x
            y_size = max_y - min_y
            mesh.apply_translation(np.array([-min_x - x_size / 2.0, -min_y - y_size / 2.0, 0]))
        elif type(step) is dict:
            for substep, substep_data in step.items():  # e.g., translate: [0, 1, 0]
                if substep == "translate":
                    mesh.apply_translation(np.array(substep_data))
                elif substep == "rotate":
                    rotation_matrix = trimesh.transformations.compose_matrix(angles=substep_data)
                    mesh.apply_transform(rotation_matrix)
                else:
                    error = f"Unknown mesh processing substep: '{substep}'."
                    raise ValueError(error)

        else:
            error = f"Unknown mesh processing step: '{step}'"
            raise ValueError(error)

    # mesh.show()

    log_info(f"Mesh loaded from path {mesh_path} has {len(mesh.vertices)} vertices.")
    simple_mesh = mesh.simplify_quadric_decimation(face_count=1000, aggression=5)
    log_info(f"Mesh after decimation has {len(simple_mesh.vertices)} vertices.")

    return mesh


class ShapeType(Enum):
    """A type of primitive 3D shape.

    Note: The Enum values correspond to the shape_msgs/SolidPrimitive types.

    Reference: https://docs.ros.org/en/noetic/api/shape_msgs/html/msg/SolidPrimitive.html
    """

    BOX = 1
    SPHERE = 2
    CYLINDER = 3
    CONE = 4

    @classmethod
    def from_string(cls, shape_type: str) -> ShapeType:
        """Construct a ShapeType instance based on the given string.

        :param shape_type: String specifying a shape type
        :return: Constructed ShapeType instance
        :raises: ValueError, if the string does not match a recognized shape type
        """
        if shape_type == "box":
            return ShapeType.BOX
        if shape_type == "sphere":
            return ShapeType.SPHERE
        if shape_type == "cylinder":
            return ShapeType.CYLINDER

        error = f"[ShapeType.from_string] Unrecognized shape type: '{shape_type}'."
        raise ValueError(error)

    @classmethod
    def get_expected_dims(cls, shape_type: ShapeType) -> int:
        """Retrieve the number of expected dimensions of the given primitive shape type."""
        if shape_type == ShapeType.BOX:
            return 3
        if shape_type == ShapeType.SPHERE:
            return 1
        if shape_type == ShapeType.CYLINDER:
            return 2

        error = f"Unrecognized shape type: {shape_type}"
        raise ValueError(error)


@dataclass
class CollisionPrimitive:
    """A primitive 3D shape representing part of an object's collision model."""

    shape_type: ShapeType
    shape_dimensions: list[float]  # Numbers characterizing the shape (e.g., radius of sphere)
    dimensions: ObjectDims  # Primitive bounding box size in (x, y, z)

    @classmethod
    def from_yaml(cls, primitive_data: dict[str, Any]) -> CollisionPrimitive:
        """Construct a CollisionPrimitive instance from a dictionary of YAML data.

        :param primitive_data: Dictionary of collision primitive data imported from YAML
        :return: Constructed CollisionPrimitive instance
        """
        shape_type = ShapeType.from_string(primitive_data["type"])
        shape_dims = primitive_data["dims"]
        expected_dims = ShapeType.get_expected_dims(shape_type)

        if len(shape_dims) != expected_dims:
            error = f"{shape_type} expects {expected_dims} dimensions, not {len(shape_dims)}."
            raise ValueError(error)

        xyz_dims = cls.find_primitive_bounding_box(shape_type, shape_dims)

        return CollisionPrimitive(
            shape_type=shape_type,
            shape_dimensions=shape_dims,
            dimensions=xyz_dims,
        )

    @classmethod
    def find_primitive_bounding_box(cls, shape_type: ShapeType, dims: list[float]) -> ObjectDims:
        """Compute the bounding box size for a primitive of the given type.

        :param shape_type: Type of the primitive shape (box/cylinder/sphere)
        :param dims: Shape-specific dimensions characterizing the primitive
        :return: Bounding box size in (x, y, z) as a tuple
        """
        if shape_type == ShapeType.BOX:  # Box dimensions: [x, y, z] (in meters)
            return (dims[0], dims[1], dims[2])

        if shape_type == ShapeType.CYLINDER:  # Cylinder dimensions: [height, radius] (in meters)
            radius_m = dims[1]
            return (2.0 * radius_m, 2.0 * radius_m, dims[0])

        if shape_type == ShapeType.SPHERE:  # Sphere dimensions: [radius] (in meters)
            diameter_m = 2.0 * dims[0]
            return (diameter_m, diameter_m, diameter_m)

        error = f"Unrecognized primitive shape: '{shape_type}'."
        raise ValueError(error)


@dataclass
class CollisionModel:
    """A collision model (primitive or mesh) for an object in the environment."""

    dimensions: ObjectDims  # Object-frame bounding box dimensions in (x, y, z), in meters
    mesh: CollisionMesh | None  # Mesh used as a collision model (optional)
    primitive: CollisionPrimitive | None  # 3D primitive shape used as a collision model (optional)

    @classmethod
    def from_yaml(cls, object_data: dict[str, Any]) -> CollisionModel:
        """Construct a CollisionModel instance from a dictionary of YAML data.

        :param object_data: YAML data specifying an object's collision model
        :return: Constructed CollisionModel instance
        :raises: KeyError, if neither a collision mesh or primitive geometry is found
        """
        mesh_data = object_data.get("mesh")
        shape_data = object_data.get("geometry")

        if mesh_data is None and shape_data is None:
            error = f"Cannot construct a CollisionModel from the YAML data: {object_data}."
            raise KeyError(error)

        mesh = None if mesh_data is None else CollisionMesh.from_yaml(mesh_data)
        primitive = None if shape_data is None else CollisionPrimitive.from_yaml(shape_data)

        mesh_dims = (0.0, 0.0, 0.0) if mesh is None else mesh.dimensions
        primitive_dims = (0.0, 0.0, 0.0) if primitive is None else primitive.dimensions
        dims_list = [max(mesh_dims[i], primitive_dims[i]) for i in range(3)]
        combined_dims = (dims_list[0], dims_list[1], dims_list[2])

        return CollisionModel(dimensions=combined_dims, mesh=mesh, primitive=primitive)

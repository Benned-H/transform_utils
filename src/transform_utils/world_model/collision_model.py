"""Define dataclasses to represent collision models for objects in the environment."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from transform_utils.filesystem.load_from_yaml_ros import load_trimesh
from transform_utils.ros.ros_utils import resolve_package_path

ObjectDims = tuple[float, float, float]  # Represents an object's size in object-frame (x,y,z)


@dataclass
class CollisionMesh:
    """An object's collision mesh, imported from file."""

    path: Path  # Path to the object's collision mesh file
    processing_steps: list[str]  # Sequence of operations to normalize the mesh (can be empty)
    dimensions: ObjectDims  # Mesh bounding box size in (x, y, z)

    @classmethod
    def from_yaml(cls, mesh_data: dict[str, Any]) -> CollisionMesh:
        """Construct a CollisionMesh instance from a dictionary of YAML data.

        :param mesh_data: Dictionary of collision mesh data imported from YAML
        :return: Constructed CollisionMesh instance
        :raises: FileNotFoundError if the mesh filepath is invalid
        """
        relative_mesh_path = mesh_data["filepath"]  # Package-relative path
        absolute_mesh_path = resolve_package_path(relative_mesh_path)

        if absolute_mesh_path is None or not absolute_mesh_path.exists():
            error_msg = f"Could not find a mesh file at the path: {relative_mesh_path}."
            raise FileNotFoundError(error_msg)

        steps = mesh_data.get("steps", [])

        loaded_trimesh = load_trimesh(absolute_mesh_path, steps)
        min_bounds, max_bounds = loaded_trimesh.bounds
        dims = tuple((max_bounds - min_bounds).tolist())

        return CollisionMesh(path=absolute_mesh_path, processing_steps=steps, dimensions=dims)


@dataclass
class CollisionPrimitive:
    """A primitive 3D shape representing part of an object's collision model."""

    shape_type: str
    shape_dimensions: list[float]  # Numbers characterizing the shape (e.g., radius of sphere)
    dimensions: ObjectDims  # Primitive bounding box size in (x, y, z)

    @classmethod
    def from_yaml(cls, primitive_data: dict[str, Any]) -> CollisionPrimitive:
        """Construct a CollisionPrimitive instance from a dictionary of YAML data.

        :param primitive_data: Dictionary of collision primitive data imported from YAML
        :return: Constructed CollisionPrimitive instance
        """
        shape_dims = primitive_data["dims"]
        shape_type = primitive_data["type"]

        if shape_type == "box":
            expected_dims = 3
        elif shape_type == "cylinder":
            expected_dims = 2
        elif shape_type == "sphere":
            expected_dims = 1
        else:
            error_msg = f"Unknown CollisionPrimitive shape type: '{shape_type}'."
            raise ValueError(error_msg)

        if len(shape_dims) != expected_dims:
            error_msg = f"{shape_type} expects {expected_dims} dimensions, not {len(shape_dims)}."
            raise ValueError(error_msg)

        xyz_dims = cls.find_primitive_bounding_box(shape_type, shape_dims)

        return CollisionPrimitive(
            shape_type=shape_type,
            shape_dimensions=shape_dims,
            dimensions=xyz_dims,
        )

    @classmethod
    def find_primitive_bounding_box(cls, shape_type: str, shape_dims: list[float]) -> ObjectDims:
        """Compute the bounding box size for a primitive of the given type.

        :param shape_type: Type of the primitive (box/cylinder/sphere)
        :param shape_dims: Shape-specific dimensions characterizing the primitive
        :return: Bounding box size in (x, y, z) as a tuple
        """
        if shape_type == "box":  # Box dimensions: [x, y, z] (in meters)
            return (shape_dims[0], shape_dims[1], shape_dims[2])

        if shape_type == "cylinder":  # Cylinder dimensions: [height, radius] (in meters)
            radius_m = shape_dims[1]
            return (2.0 * radius_m, 2.0 * radius_m, shape_dims[0])

        if shape_type == "sphere":  # Sphere dimensions: [radius] (in meters)
            diameter_m = 2.0 * shape_dims[0]
            return (diameter_m, diameter_m, diameter_m)

        error_msg = f"Unrecognized primitive shape: '{shape_type}'."
        raise ValueError(error_msg)


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
        :raises: KeyError if neither a collision mesh or primitive geometry is found
        """
        mesh_data = object_data.get("mesh")
        shape_data = object_data.get("geometry")

        if mesh_data is None and shape_data is None:
            error_msg = f"Cannot construct a CollisionModel from the YAML data: {object_data}."
            raise KeyError(error_msg)

        mesh = None if mesh_data is None else CollisionMesh.from_yaml(mesh_data)
        primitive = None if shape_data is None else CollisionPrimitive.from_yaml(shape_data)

        mesh_dims = (0.0, 0.0, 0.0) if mesh is None else mesh.dimensions
        primitive_dims = (0.0, 0.0, 0.0) if primitive is None else primitive.dimensions
        dims_list = [max(mesh_dims[i], primitive_dims[i]) for i in range(3)]
        combined_dims = (dims_list[0], dims_list[1], dims_list[2])

        return CollisionModel(dimensions=combined_dims, mesh=mesh, primitive=primitive)

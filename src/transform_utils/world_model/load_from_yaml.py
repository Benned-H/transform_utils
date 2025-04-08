"""Define functions for loading object and environment models from YAML."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import geometry_msgs.msg
import numpy as np
import shape_msgs.msg
import trimesh
import yaml
from moveit_msgs.msg import CollisionObject

from transform_utils.kinematics import DEFAULT_FRAME, Pose2D, Pose3D
from transform_utils.kinematics_ros import pose_to_msg
from transform_utils.logging import log_error, log_info
from transform_utils.ros.ros_utils import resolve_package_path
from transform_utils.world_model.object_model import ObjectModel

if TYPE_CHECKING:
    from pathlib import Path


class ImportedMeshTypeError(Exception):
    """An error caused by an unexpected mesh type during a Trimesh import."""

    def __init__(self, mesh_path: Path, mesh_type: type):
        """Initialize the error with the path and type of the imported mesh."""
        super().__init__(f"{mesh_path} imported as unexpected type: {mesh_type}.")


@dataclass
class EnvironmentModel:
    """An environment model loaded from YAML."""

    base_poses: dict[str, Pose3D]  # Maps robot names to their initial base poses
    objects: dict[str, ObjectModel]  # Maps object names to their geometric models


def load_yaml_into_dict(yaml_path: Path) -> dict[str, Any]:
    """Load data from a YAML file into a Python dictionary.

    :param yaml_path: Path to the YAML file to be imported
    :return: Dictionary mapping strings to values (empty if the YAML file is nonexistent/invalid)
    """
    if not yaml_path.exists():
        log_error(f"The YAML path {yaml_path} doesn't exist!")
        return {}

    try:
        with yaml_path.open() as yaml_file:
            yaml_data = yaml.safe_load(yaml_file)
            log_info(f"Loaded data from YAML file: {yaml_path}")

    except yaml.YAMLError as error:
        log_error(f"Failed to load YAML file: {yaml_path}\nError: {error}")
        return {}

    return yaml_data


def load_named_poses_2d(landmarks_data: list[Any], default_frame: str) -> dict[str, Pose2D]:
    """Load a collection of named 2D poses from data imported from YAML.

    :param landmarks_data: List of dictionaries, each mapping a location name to 2D pose data
    :param default_frame: Reference frame used for any poses without a specified frame
    :return: Map from robot/object/location names to 2D poses
    """
    named_poses: dict[str, Pose2D] = {}

    for landmark_data in landmarks_data:
        for name, pose_data in landmark_data.items():
            if isinstance(pose_data, list):  # Pose represented as [x, y, yaw] list
                named_poses[name] = Pose2D.from_list(pose_data, ref_frame=default_frame)
            elif isinstance(pose_data, dict):  # Pose with reference frame specified
                ref_frame: str = pose_data.get("frame", default_frame)
                pose_list = pose_data["x_y_yaw"]
                named_poses[name] = Pose2D.from_list(pose_list, ref_frame=ref_frame)

    return named_poses


def load_named_poses(poses_data: dict[str, Any], default_frame: str) -> dict[str, Pose3D]:
    """Load a collection of named 3D poses from data imported from YAML.

    :param poses_data: Dictionary mapping robot/object/location names to 3D pose data
    :param default_frame: Reference frame used for any poses without a specified frame
    :return: Map from robot/object/location names to 3D poses
    """
    named_poses: dict[str, Pose3D] = {}

    for name, pose_data in poses_data.items():
        if isinstance(pose_data, list):  # Pose represented as XYZ-RPY list
            named_poses[name] = Pose3D.from_list(pose_data, ref_frame=default_frame)
        elif isinstance(pose_data, dict):  # Pose with reference frame specified
            ref_frame: str = pose_data.get("frame", default_frame)
            pose_list = pose_data["xyz_rpy"]
            named_poses[name] = Pose3D.from_list(pose_list, ref_frame=ref_frame)

    return named_poses


def load_environment_from_yaml(yaml_path: Path) -> EnvironmentModel:
    """Load an environment model from the given YAML file.

    :param yaml_path: Path to the YAML file containing an environment model
    :return: Environment model consisting of loaded robot poses and objects
    """
    env = EnvironmentModel({}, {})

    yaml_data = load_yaml_into_dict(yaml_path)

    if "robot_base_poses" in yaml_data:
        env.base_poses = load_named_poses(yaml_data["robot_base_poses"], DEFAULT_FRAME)

    if "objects" in yaml_data:
        env.objects = load_objects(yaml_data["objects"])

    return env


def load_objects(objects_data: dict[str, dict[str, Any]]) -> dict[str, ObjectModel]:
    """Load objects from data imported from YAML.

    :param objects_data: Dictionary mapping object names to object data
    :return: Dictionary mapping object names to object models
    """
    return {name: load_object(name, data) for (name, data) in objects_data.items()}


def load_object(object_name: str, object_data: dict[str, Any]) -> ObjectModel:
    """Load an object model from data imported from YAML.

    Note: This method sets the stored CollisionObject message's `operation` field to ADD.

    :param object_name: Name of the object being loaded
    :param object_data: Dictionary mapping YAML field names to object data
    :return: Constructed ObjectModel
    """
    collision_object_msg = CollisionObject()

    # Initialize the object's pose if it's specified in the YAML data
    pose_data = object_data.get("pose")
    ref_frame = object_data.get("frame", DEFAULT_FRAME)
    collision_object_msg.header.frame_id = ref_frame

    # If no pose is provided, default the ROS message to the identity pose in the default frame
    if pose_data is None:
        pose = None
        collision_object_msg.pose = pose_to_msg(Pose3D.identity())
    else:
        pose = Pose3D.from_list(pose_data, ref_frame)
        collision_object_msg.pose = pose_to_msg(pose)

    collision_object_msg.id = object_name

    object_type = object_data["type"]
    collision_object_msg.type.key = object_type  # Ignore 'db' field of message

    # Verify that the object's geometry is specified either as a solid primitive or as a mesh
    assert "mesh" in object_data or "geometry" in object_data

    if "mesh" in object_data:
        relative_mesh_path = object_data["mesh"]["filepath"]

        # Resolve the mesh's package-relative filepath as an absolute path
        mesh_path = resolve_package_path(relative_mesh_path)
        assert mesh_path is not None, (
            f"Could not resolve the package-relative path: {relative_mesh_path}"
        )

        mesh_steps = object_data["mesh"]["steps"]  # Steps to process the mesh

        loaded_trimesh = load_trimesh(mesh_path, mesh_steps)

        # Set the mesh pose to the identity pose so that object frame = mesh frame
        collision_object_msg.meshes = [trimesh_to_msg(loaded_trimesh)]
        collision_object_msg.mesh_poses = [geometry_msgs.msg.Pose()]

        # Find object dimensions using the imported mesh
        min_bounds, max_bounds = loaded_trimesh.bounds
        object_dims = max_bounds - min_bounds

    if "geometry" in object_data:
        solid_primitive_msg, object_dims = load_solid_primitive(object_data["geometry"])
        collision_object_msg.primitives.append(solid_primitive_msg)

        # SolidPrimitive messages place their geometry at the origin of the frame,
        #   whereas we want the object's frame (obj) at the bottom of the geometry (geo)
        height_m = object_dims[2]
        pose_o_g = Pose3D.from_xyz_rpy(z=height_m / 2.0)
        collision_object_msg.primitive_poses.append(pose_to_msg(pose_o_g))

    # Create any subframes specified in the YAML data
    subframes = object_data.get("subframes", [])
    for subframe_name in subframes:
        if subframe_name == "top":
            subframe_pose = Pose3D.from_xyz_rpy(z=object_dims[2])

            collision_object_msg.subframe_names.append(subframe_name)
            collision_object_msg.subframe_poses.append(subframe_pose)

    collision_object_msg.operation = CollisionObject.ADD

    log_info(f"Loaded object named '{object_name}' within load_object()...")

    return ObjectModel(collision_object_msg, pose, object_dims)


def load_solid_primitive(
    primitive_data: dict[str, Any],
) -> tuple[shape_msgs.msg.SolidPrimitive, tuple[float, float, float]]:
    """Load a shape_msgs/SolidPrimitive message and its dimensions from YAML data.

    These primitives include boxes, spheres, cylinders, and cones. The SolidPrimitive
        message models all shapes as having bounding boxes centered at [0,0,0].

    Reference: https://docs.ros.org/en/noetic/api/shape_msgs/html/msg/SolidPrimitive.html

    :param primitive_data: Dictionary mapping YAML field names to shape data
    :return: Tuple containing a SolidPrimitive message and the object's dimensions
    """
    msg = shape_msgs.msg.SolidPrimitive()
    msg.dimensions = primitive_data["dims"]
    shape_type = primitive_data["type"]

    if shape_type == "box":
        msg.type = shape_msgs.msg.SolidPrimitive.BOX
        expected_dims = 3  # Box primitive dimensions: [x, y, z]

    elif shape_type == "cylinder":
        msg.type = shape_msgs.msg.SolidPrimitive.CYLINDER
        expected_dims = 2  # Cylinder primitive dimensions: [height, radius] (centered on z-axis)

    elif shape_type == "sphere":
        msg.type = shape_msgs.msg.SolidPrimitive.SPHERE
        expected_dims = 1  # Sphere primitive dimensions: [radius] (centered on origin)

    actual_dims = len(msg.dimensions)
    assert actual_dims == expected_dims, f"{shape_type} shape expects {expected_dims} dimensions!"

    if shape_type == "box":
        shape_dims = (
            msg.dimensions[shape_msgs.msg.SolidPrimitive.BOX_X],
            msg.dimensions[shape_msgs.msg.SolidPrimitive.BOX_Y],
            msg.dimensions[shape_msgs.msg.SolidPrimitive.BOX_Z],
        )
    elif shape_type == "cylinder":
        shape_dims = (
            2.0 * msg.dimensions[shape_msgs.msg.SolidPrimitive.CYLINDER_RADIUS],
            2.0 * msg.dimensions[shape_msgs.msg.SolidPrimitive.CYLINDER_RADIUS],
            msg.dimensions[shape_msgs.msg.SolidPrimitive.CYLINDER_HEIGHT],
        )
    elif shape_type == "sphere":
        shape_dims = (
            2.0 * msg.dimensions[shape_msgs.msg.SolidPrimitive.SPHERE_RADIUS],
            2.0 * msg.dimensions[shape_msgs.msg.SolidPrimitive.SPHERE_RADIUS],
            2.0 * msg.dimensions[shape_msgs.msg.SolidPrimitive.SPHERE_RADIUS],
        )

    return (msg, shape_dims)


def load_trimesh(mesh_path: Path, import_steps: list[str | dict]) -> trimesh.Trimesh:
    """Load an object's mesh from file.

    During import, the operations prescribed by "steps" are applied to the mesh.

    :param mesh_path: Path to the mesh file to be imported
    :param import_steps: Sequence of geometric operations applied to the mesh (e.g., "center_xy")
    :return: Normalized mesh imported using the trimesh library
    """
    mesh = trimesh.load(mesh_path, force="mesh")
    if type(mesh) is not trimesh.Trimesh:
        raise ImportedMeshTypeError(mesh_path, type(mesh))

    mesh = mesh.convert_units("meters", guess=True)

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
                    raise ValueError(f"Unknown mesh processing substep: '{substep}'")

        else:
            raise ValueError(f"Unknown mesh processing step: '{step}'")

    mesh.show()

    return mesh


def trimesh_to_msg(mesh: trimesh.Trimesh) -> shape_msgs.msg.Mesh:
    """Convert a trimesh.Trimesh into a shape_msgs/Mesh message.

    :param mesh: Object containing a 3D triangle mesh
    :return: A shape_msgs/Mesh message containing the mesh geometry
    """
    mesh_msg = shape_msgs.msg.Mesh()
    mesh_msg.triangles = [shape_msgs.msg.MeshTriangle(list(tri)) for tri in mesh.faces]
    mesh_msg.vertices = [geometry_msgs.msg.Point(v[0], v[1], v[2]) for v in mesh.vertices]

    return mesh_msg

"""Define functions for loading object and environment models from YAML."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import geometry_msgs.msg
import numpy as np
import rospy
import shape_msgs.msg
import trimesh
import yaml
from moveit_msgs.msg import CollisionObject

from transform_utils.kinematics import DEFAULT_FRAME, Pose3D
from transform_utils.kinematics_ros import pose_to_msg
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
class ObjectDataYAML:
    """A dataclass to store data about an object loaded from YAML."""

    model: ObjectModel
    pose: Pose3D | None


@dataclass
class EnvironmentModel:
    """An environment model loaded from YAML."""

    base_poses: dict[str, Pose3D]  # Maps robot names to their initial base poses
    objects: dict[str, ObjectDataYAML]  # Maps object names to their models and initial poses


def load_known_object_names_from_yaml(yaml_path: Path) -> list[str]:
    """Load the list of known object names from the given YAML file.

    :param yaml_path: Path to the YAML file containing an environment model
    :return: List of names of the known objects in the environment
    """
    env = load_environment_from_yaml(yaml_path)
    return list(env.objects.keys())


def load_environment_from_yaml(yaml_path: Path) -> EnvironmentModel:
    """Load an environment model from the given YAML file.

    :param yaml_path: Path to the YAML file containing an environment model
    :return: Environment model consisting of loaded robot poses and objects
    """
    env = EnvironmentModel({}, {})

    if not yaml_path.exists():
        rospy.logerr(f"The YAML path {yaml_path} doesn't exist!")
        return env

    try:
        with yaml_path.open() as yaml_file:
            yaml_data = yaml.safe_load(yaml_file)
            rospy.loginfo(f"Loaded environment data from YAML file: {yaml_path}")
    except yaml.YAMLError as error:
        rospy.logerr(f"Failed to load YAML file: {yaml_path}\nError: {error}")
        return env

    if "robot_base_poses" in yaml_data:
        env.base_poses = load_robot_base_poses(yaml_data["robot_base_poses"])

    if "objects" in yaml_data:
        env.objects = load_objects(yaml_data["objects"])

    return env


def load_robot_base_poses(robots_data: dict[str, list[float]]) -> dict[str, Pose3D]:
    """Load robots' initial base poses from data imported from YAML.

    :param robots_data: Map from robot names to base poses (lists of XYZ RPY floats)
    :return: Map from robot names to robot base poses represented as Pose3Ds
    """
    base_poses: dict[str, Pose3D] = {}

    for robot_name, base_pose_list in robots_data.items():
        base_poses[robot_name] = Pose3D.from_list(base_pose_list, DEFAULT_FRAME)

    return base_poses


def load_objects(objects_data: dict[str, dict[str, Any]]) -> dict[str, ObjectDataYAML]:
    """Load objects from data imported from YAML.

    :param objects_data: Dictionary mapping object names to object data
    :return: Dataclass storing object models and their estimated poses
    """
    return {name: load_object(name, object_data) for name, object_data in objects_data.items()}


def load_object(object_name: str, object_data: dict[str, Any]) -> ObjectDataYAML:
    """Load an object model and 3D pose from data imported from YAML.

    Note: This method sets the stored CollisionObject message's `operation` field to ADD.

    :param object_name: Name of the object being loaded
    :param object_data: Dictionary mapping YAML field names to object data
    :return: Constructed ObjectModel and initial object pose (or None if no pose is available)
    """
    collision_object_msg = CollisionObject()

    # Initialize the object's pose if it's specified in the YAML data
    pose_data = object_data.get("pose")
    ref_frame = object_data.get("frame", DEFAULT_FRAME)
    collision_object_msg.header.frame_id = ref_frame

    collision_object_msg.id = object_name

    static = object_data.get("static", False)
    print(f"Whether {object_name} is static was loaded as: {static} of type {type(static)}")

    # If no pose is provided, default the ROS message to the identity pose in the default frame
    if pose_data is None:
        pose = None
        collision_object_msg.pose = pose_to_msg(Pose3D.identity())
    else:
        pose = Pose3D.from_list(pose_data, ref_frame)
        collision_object_msg.pose = pose_to_msg(pose)

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

        mesh_steps = object_data["mesh"].get("steps", [])  # Steps to process the mesh

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
    object_model = ObjectModel(collision_object_msg, object_dims, static)

    rospy.loginfo(f"Loaded object named '{object_name}' within load_object()...")

    return ObjectDataYAML(object_model, pose)


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

    # mesh.show()

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

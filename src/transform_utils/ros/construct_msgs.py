"""Define utility functions to construct ROS messages from existing data structures."""

import geometry_msgs.msg
import shape_msgs.msg
import trimesh
from moveit_msgs.msg import CollisionObject

from transform_utils.kinematics import Pose3D
from transform_utils.kinematics_ros import pose_to_msg
from transform_utils.states.collision_models import CollisionPrimitive
from transform_utils.states.object_state import ObjectState


def make_collision_object_msg(object_state: ObjectState) -> CollisionObject:
    """Construct a moveit_msgs/CollisionObject message using the given data.

    :param object_state: Object model specifying the object's state
    :return: Constructed moveit_msgs/CollisionObject message
    """
    msg = CollisionObject()
    msg.header.frame_id = object_state.pose.ref_frame
    msg.pose = pose_to_msg(object_state.pose)

    msg.id = object_state.name
    msg.type.key = object_state.object_type  # Ignore 'db' field of message

    if object_state.collision_model.mesh is not None:
        msg.meshes.append(trimesh_to_msg(object_state.collision_model.mesh.mesh))
        msg.mesh_poses.append(geometry_msgs.msg.Pose())

    if object_state.collision_model.primitive is not None:
        msg.primitives.append(collision_primitive_to_msg(object_state.collision_model.primitive))

        # SolidPrimitive messages place their geometry centered at the origin of the frame,
        #   whereas we want the object's frame (o) at the bottom of the geometry (g)
        height_m = object_state.collision_model.primitive.dimensions[2]
        pose_o_g = Pose3D.from_xyz_rpy(z=height_m / 2.0)
        msg.primitive_poses.append(pose_to_msg(pose_o_g))

    # TODO: Could add subframes back, skipping for now

    msg.operation = CollisionObject.ADD

    return msg


def trimesh_to_msg(mesh: trimesh.Trimesh) -> shape_msgs.msg.Mesh:
    """Convert a trimesh.Trimesh into a shape_msgs/Mesh message.

    :param mesh: 3D triangle mesh
    :return: shape_msgs/Mesh message containing the mesh geometry
    """
    mesh_msg = shape_msgs.msg.Mesh()
    mesh_msg.triangles = [shape_msgs.msg.MeshTriangle(list(tri)) for tri in mesh.faces]
    mesh_msg.vertices = [geometry_msgs.msg.Point(v[0], v[1], v[2]) for v in mesh.vertices]

    return mesh_msg


def collision_primitive_to_msg(primitive: CollisionPrimitive) -> shape_msgs.msg.SolidPrimitive:
    """Convert a CollisionPrimitive instance into a shape_msgs/SolidPrimitive message.

    Reference: https://docs.ros.org/en/noetic/api/shape_msgs/html/msg/SolidPrimitive.html

    :param primitive: Primitive 3D shape representing part of a collision model
    :return: Constructed shape_msgs/SolidPrimitive message
    """
    msg = shape_msgs.msg.SolidPrimitive()
    msg.dimensions = list(primitive.dimensions)
    msg.type = int(primitive.shape_type)

    return msg

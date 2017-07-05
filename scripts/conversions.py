from __future__ import print_function

# Based on source code Developed by: andreasBihlmaier
# source:   https://github.com/andreasBihlmaier/gazebo2rviz
#           https://github.com/andreasBihlmaier/pysdf

import copy
import os.path
import rospy
from rospkg import RosPack, ResourceNotFound
from visualization_msgs.msg import Marker
import numbers

from tf.transformations import *
from geometry_msgs.msg import Pose

from naming import *

models_paths = [os.path.expanduser('~/.gazebo/models/')]

if 'GAZEBO_MODEL_PATH' in os.environ:
    models_paths = os.environ['GAZEBO_MODEL_PATH'].split(':')

protoMarkerMsg = Marker()
protoMarkerMsg.frame_locked = True
protoMarkerMsg.id = 0
protoMarkerMsg.action = Marker.ADD
protoMarkerMsg.mesh_use_embedded_materials = True
protoMarkerMsg.color.a = 0.0
protoMarkerMsg.color.r = 0.0
protoMarkerMsg.color.g = 0.0
protoMarkerMsg.color.b = 0.0
supported_geometry_types = ['mesh', 'cylinder', 'sphere', 'box']

gazebo_rospack = RosPack()


def link2marker_msg(link, full_linkname, use_collision=False, lifetime=rospy.Duration(0)):
    marker_msg = None
    linkpart = None
    if use_collision:
        linkparts = getattr(link, 'collisions')
    else:  # visual
        linkparts = getattr(link, 'visuals')

    msgs = []

    for linkpart in linkparts:
        if not linkpart.geometry_type in supported_geometry_types:
            if linkpart.geometry_type:
                print("Element %s with geometry type %s not supported. Ignored." % (
                    full_linkname, linkpart.geometry_type))
                return None

        marker_msg = copy.deepcopy(protoMarkerMsg)
        marker_msg.header.frame_id = sdf2tfname(full_linkname)
        marker_msg.header.stamp = rospy.get_rostime()
        marker_msg.lifetime = lifetime
        marker_msg.ns = sdf2tfname(full_linkname + "::" + linkpart.name)
        marker_msg.pose = homogeneous2pose_msg(linkpart.pose)

        if linkpart.geometry_type == 'mesh':
            marker_msg.type = Marker.MESH_RESOURCE
            # print('linkpart.geometry_data: %s' % linkpart.geometry_data['uri'])
            for models_path in models_paths:
                resource = linkpart.geometry_data['uri'].replace('model://', models_path + '/')
                # print('resource: %s' % resource)
                if os.path.isfile(resource):
                    marker_msg.mesh_resource = 'file://' + resource
                    # print('found resource %s at %s' % (linkpart.geometry_data['uri'], resource))
                    break
            # support URDF-like resource paths starting with model://
            if not marker_msg.mesh_resource and linkpart.geometry_data['uri'].startswith('model://'):
                stripped_uri = linkpart.geometry_data['uri'].replace('model://', '')
                uri_parts = stripped_uri.split('/', 1)

                if len(uri_parts) == 2:
                    package_name = uri_parts[0]
                    try:
                        package_path = gazebo_rospack.get_path(package_name)
                        mesh_path = os.path.join(package_path, uri_parts[1])
                        if os.path.isfile(mesh_path):
                            marker_msg.mesh_resource = 'file://' + mesh_path
                    except ResourceNotFound, e:
                        pass

            if not marker_msg.mesh_resource:
                print('ERROR! could not find resource: %s' % linkpart.geometry_data['uri'])
                return None

            scale = (float(val) for val in linkpart.geometry_data['scale'].split())
            marker_msg.scale.x, marker_msg.scale.y, marker_msg.scale.z = scale
        else:
            marker_msg.color.a = 1
            marker_msg.color.r = marker_msg.color.g = marker_msg.color.b = 0.5

        if linkpart.geometry_type == 'box':
            marker_msg.type = Marker.CUBE
            scale = (float(val) for val in linkpart.geometry_data['size'].split())
            marker_msg.scale.x, marker_msg.scale.y, marker_msg.scale.z = scale
        elif linkpart.geometry_type == 'sphere':
            marker_msg.type = Marker.SPHERE
            marker_msg.scale.x = marker_msg.scale.y = marker_msg.scale.z = 2.0 * float(linkpart.geometry_data['radius'])
        elif linkpart.geometry_type == 'cylinder':
            marker_msg.type = Marker.CYLINDER
            marker_msg.scale.x = marker_msg.scale.y = 2.0 * float(linkpart.geometry_data['radius'])
            marker_msg.scale.z = float(linkpart.geometry_data['length'])

        # print(marker_msg)
        msgs.append(marker_msg)
    return msgs


def rounded(val):
    if isinstance(val, str):
        return rounded(float(val))
    elif isinstance(val, numbers.Number):
        return int(round(val, 6) * 1e5) / 1.0e5
    else:
        return numpy.array([rounded(v) for v in val])


def homogeneous2translation_quaternion(homogeneous):
    """
    Translation: [x, y, z]
    Quaternion: [x, y, z, w]
    """
    translation = translation_from_matrix(homogeneous)
    quaternion = quaternion_from_matrix(homogeneous)
    return translation, quaternion


def homogeneous2translation_rpy(homogeneous):
    """
    Translation: [x, y, z]
    RPY: [sx, sy, sz]
    """
    translation = translation_from_matrix(homogeneous)
    rpy = euler_from_matrix(homogeneous)
    return translation, rpy


def homogeneous2pose_msg(homogeneous):
    pose = Pose()
    translation, quaternion = homogeneous2translation_quaternion(homogeneous)
    pose.position.x = translation[0]
    pose.position.y = translation[1]
    pose.position.z = translation[2]
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    return pose


def pose_msg2homogeneous(pose):
    trans = translation_matrix((pose.position.x, pose.position.y, pose.position.z))
    rot = quaternion_matrix((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
    return concatenate_matrices(trans, rot)


def array2string(array):
    return numpy.array_str(array).strip('[]. ').replace('. ', ' ')


def homogeneous2tq_string(homogeneous):
    return 't=%s q=%s' % homogeneous2translation_quaternion(homogeneous)


def homogeneous2tq_string_rounded(homogeneous):
    return 't=%s q=%s' % tuple(rounded(o) for o in homogeneous2translation_quaternion(homogeneous))


def string2float_list(s):
    return [float(i) for i in s.split()]


def pose_string2homogeneous(pose):
    pose_float = string2float_list(pose)
    translate = pose_float[:3]
    angles = pose_float[3:]
    homogeneous = compose_matrix(None, None, angles, translate)
    # print('pose_string=%s; translate=%s angles=%s homogeneous:\n%s' % (pose, translate, angles, homogeneous))
    return homogeneous


def rotation_only(homogeneous):
    euler = euler_from_matrix(homogeneous)
    return euler_matrix(euler[0], euler[1], euler[2])

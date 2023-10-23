#!/usr/bin/env python

"""
Starter script for Project 1B
Author: Chris Correa, Han Nguyen
"""
import numpy as np
from math import sin, cos, atan2


try:
    from geometry_msgs.msg._Point import Point
    import tf.transformations as tfs
    from geometry_msgs.msg import Pose, PoseStamped
    ros_enabled = True
except:
    ros_enabled = False

def length(vec):
    """
    Returns the length of a 1 dimensional numpy vector

    Parameters
    ----------
    vec : nx1 :obj:`numpy.ndarray`

    Returns
    -------
    float
        ||vec||_2^2
    """
    return np.linalg.norm(vec)

def normalize(vec):
    """
    Returns a normalized version of a numpy vector

    Parameters
    ----------
    vec : nx' :obj:`numpy.ndarray

    Returns
    -------
    nx' :obj:`numpy.ndarray`
    """
    return vec / length(vec)

def joint_array_to_dict(vel_torque_array, limb):
    """
    the sawyer interface requires you to send the joint velocities / torques
    as a dictionary, this turns and array of velocities and torques into a 
    dictionary with joint names.

    Parameters
    ----------
    vel_torque_array : 7x' :obj:`numpy.ndarray`
        numpy array of velocities or torques to be sent to the sawyer
    limb : :obj:`sawyer_interface.Limb`
        Limb object

    Returns
    -------
    :obj:`dict` of string->float
        mapping of joint names to joint velocities / torques
    """

    return dict(zip(limb.joint_names(), vel_torque_array))

def get_joint_positions(limb):
    """
    Returns the sawyer joint positions IN ORDER (execute order 66)

    Parameters
    ----------
    limb : :obj:`sawyer_interface.Limb`
        Limb object

    Returns
    -------
    7x' :obj:`numpy.ndarray`
        joint positions
    """
    return np.array([limb.joint_angles()[joint_name] for joint_name in limb.joint_names()])

def get_joint_velocities(limb):
    """
    Returns the sawyer joint velocities IN ORDER (execute order 66)

    Parameters
    ----------
    limb : :obj:`sawyer_interface.Limb`
        Limb object

    Returns
    -------
    7x' :obj:`numpy.ndarray`
        joint velocities
    """
    return np.array([limb.joint_velocities()[joint_name] for joint_name in limb.joint_names()])

def vec(*args):
    """
    all purpose function to get a numpy array of random things.  you can pass
    in a list, tuple, ROS Point message.  you can also pass in:
    vec(1,2,3,4,5,6) which will return a numpy array of each of the elements 
    passed in: np.array([1,2,3,4,5,6])
    """
    if len(args) == 1:
        if type(args[0]) == tuple:
            return np.array(args[0])
        elif ros_enabled and type(args[0]) == Point:
            return np.array((args[0].x, args[0].y, args[0].z))
        else:
            return np.array(args)
    else:
        return np.array(args)

def hat(v):
    """
    See https://en.wikipedia.org/wiki/Hat_operator or the MLS book

    Parameters
    ----------
    v : :obj:`numpy.ndarrray`
        vector form of shape 3x1, 3x, 6x1, or 6x

    Returns
    -------
    3x3 or 6x6 :obj:`numpy.ndarray`
        hat version of the vector v
    """
    if v.shape == (3, 1) or v.shape == (3,):
        return np.array([
                [0, -v[2], v[1]],
                [v[2], 0, -v[0]],
                [-v[1], v[0], 0]
            ])
    elif v.shape == (6, 1) or v.shape == (6,):
        return np.array([
                [0, -v[5], v[4], v[0]],
                [v[5], 0, -v[3], v[1]],
                [-v[4], v[3], 0, v[2]],
                [0, 0, 0, 0]
            ])
    else:
        raise ValueError

def axis_angle(quat1, quat2):
    """
    Computes the angle between the configurations described by the
    two quaternions.
    """
    R1 = tfs.quaternion_matrix(quat1)[:3, :3]
    R2 = tfs.quaternion_matrix(quat2)[:3, :3]
    R = np.matmul(R1.T, R2) 
    tr_R = sum(R[i, i] for i in range(3))
    theta = np.arccos((tr_R - 1) / 2.0)
    return theta

def get_g_matrix(position, quat):
    g = np.eye(4)
    g[:3, :3] = tfs.quaternion_matrix(quat)[:3, :3]
    g[:3, 3] = position
    return g

def adj(g):
    """
    Adjoint of a rotation matrix.  See the MLS book

    Parameters
    ----------
    g : 4x4 :obj:`numpy.ndarray`
        Rotation matrix

    Returns
    -------
    6x6 :obj:`numpy.ndarray` 
    """
    if g.shape != (4, 4):
        raise ValueError

    R = g[0:3,0:3]
    p = g[0:3,3]
    result = np.zeros((6, 6))
    result[0:3,0:3] = R
    result[0:3,3:6] = hat(p) * R
    result[3:6,3:6] = R
    return result

def twist_from_tf(g):
    """
    Returns the twist version of a 2D rotation matrix
    Parameters
    ----------
    g : 3x3 :obj:`numpy.ndarray`
        2D rotation matrix

    Returns
    -------
    3x' :obj:`numpy.ndarray`
    """
    return vec(g[0,2], g[1,2], atan2(g[1,0], g[0,0]))

def rotation2d(theta):
    """
    2D rotation matrix from a single theta around the origin

    Parameters
    ----------
    theta : float

    Returns
    -------
    2x2 :obj:`numpy.ndarray`
    """
    return np.array([
            [cos(theta), -sin(theta)],
            [sin(theta), cos(theta)]
        ])

def rigid(twist):
    """
    Returns a 3x3 Rotation Matrix version of a 2D twist

    Parameters
    ----------
    twist : 3x1 :obj:`numpy.ndarray`

    Returns
    -------
    3x3 :obj:`numpy.ndarray`
    """
    return np.array([
            [cos(twist[2]), -sin(twist[2]), twist[0]],
            [sin(twist[2]), cos(twist[2]), twist[1]],
            [0, 0, 1]
        ])

def look_at_general(origin, direction):
    """
    Creates a 3D Rotation Matrix at the origin such that the z axis is the same
    as the direction specified.  There are infinitely many of such matrices, 
    but we choose the one where the x axis is as vertical as possible.  

    Parameters
    ----------
    origin : 3x1 :obj:`numpy.ndarray`
    direction : 3x1 :obj:`numpy.ndarray`

    Returns
    -------
    4x4 :obj:`numpy.ndarray`
    """
    up = vec(0,0,1)
    z = normalize(direction)
    x = normalize(np.cross(up, z))
    y = np.cross(z, x) 

    result = np.eye(4)
    result[0:3,0] = x
    result[0:3,1] = y
    result[0:3,2] = z
    result[0:3,3] = origin
    return result

def create_pose_from_rigid_transform(g):
    """
    takes a rotation matrix and turns it into a ROS Pose

    Parameters
    ----------
    g : 4x4 : :obj:`numpy.ndarray`

    Returns
    -------
    :obj:`geometry_msgs.msg.Pose`
    """
    position = tfs.translation_from_matrix(g)
    quaternion = tfs.quaternion_from_matrix(g)
    wpose = Pose()
    wpose.position.x = position[0]
    wpose.position.y = position[1]
    wpose.position.z = position[2]
    wpose.orientation.x = quaternion[0]
    wpose.orientation.y = quaternion[1]
    wpose.orientation.z = quaternion[2]
    wpose.orientation.w = quaternion[3]
    return wpose

def create_pose_stamped_from_pos_quat(pos, quat, frame_id):
    """
    takes a position and quaternion and turns it into a ROS PoseStamped

    Parameters
    ----------
    pos : 3x1 : :obj:`numpy.ndarray`
    quat : 4x1 : :obj:`numpy.ndarray`


    Returns
    -------
    :obj:`geometry_msgs.msg.PoseStamped`
    """
    wpose = PoseStamped()
    wpose.header.frame_id = frame_id
    wpose.pose.position.x = pos[0]
    wpose.pose.position.y = pos[1]
    wpose.pose.position.z = pos[2]
    wpose.pose.orientation.x = quat[0]
    wpose.pose.orientation.y = quat[1]
    wpose.pose.orientation.z = quat[2]
    wpose.pose.orientation.w = quat[3]
    return wpose

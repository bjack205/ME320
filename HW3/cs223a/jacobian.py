#!/usr/bin/env python
"""
jacobian.py

Authors: Toki Migimatsu
         Vikranth Dwaracherla
         Mingyu Wang
Created: December 2017
"""

import numpy as np
from links import *

#####################
# HW3 Q2: Jacobians #
#####################

# TODO: Implement 3 functions below

def linear_jacobian(links, q, pos_in_link=None, link_frame=-1):
    """
    Computes the linear Jacobian J_v for the current configuration q.

    This Jacobian relates joint velocities to linear velocities at a specified
    point on the robot. The location of this point is characterized by the robot
    link it is attached to (link_frame) and the position of the point in that
    link (pos_in_link).

    Args:
        links   [Link, ...]: list of Link objects (see links.py)
        q           [N x 1]: Numpy array of joint values (radians for revolute joints)
        pos_in_link [3 x 1]: Numpy array for the position of the Jacobian point in link_frame
        link_frame    (int): frame index of the Jacobian point (-1 for the last link)

    Returns:
        [3 x N] Numpy array
    """
    # Set the default pos_in_link to [0 0 0]
    if pos_in_link is None:
        pos_in_link = np.zeros(3)
    # Make negative link_frame indices positive
    if link_frame < 0:
        link_frame += len(links)
    # Check parameters
    if len(links) != q.shape[0]:
        raise ValueError("Number of links (%d) and joints q (%d) must be equal." % (len(links), q.shape[0]))
    if pos_in_link.shape[0] != 3:
        raise ValueError("Position pos_in_link must be of dimension 3.")
    if link_frame < 0 or link_frame >= len(links):
        raise ValueError("Link frame index link_frame must be in the range [0, N-1].")

    # Set up variables
    dof = q.shape[0]
    eps = prismatic_joints(links)[np.newaxis,:link_frame+1] # eps = 1 if prismatic, 0 if revolute
    J_v = np.zeros((3, dof))


    # TODO: Replace the following line with your code
    T0 = T_all_to_0(links, q)
    pe = T0[link_frame][0:3,3] + T0[link_frame][0:3,0:3].dot(pos_in_link)
    for l in range(link_frame+1):
        z = T0[l][0:3,2]  # z-axis in world frame
        p = T0[l][0:3,3]  # translation in world frame
        J_v[:,l] = eps[0,l]*z + (1-eps[0,l])*np.cross(z, pe-p)

    return J_v


def angular_jacobian(links, q, link_frame=-1):
    """
    Computes the angular Jacobian J_w for the current configuration q.

    This Jacobian relates joint velocities to angular velocities at a specified
    point on the robot. The location of this point is characterized by the robot
    link it is attached to (link_frame). The position of the point in the link
    does not matter.

    Args:
        links [Link, ...]: list of Link objects (see links.py)
        q         [N x 1]: Numpy array of joint values (radians for revolute joints)
        link_frame  (int): frame index of Jacobian point (-1 for the last link)

    Returns:
        [3 x N] Numpy array
    """
    # Make negative link_frame indices positive
    if link_frame < 0:
        link_frame += len(links)
    # Check parameters
    if len(links) != q.shape[0]:
        raise ValueError("Number of links (%d) and joints q (%d) must be equal." % (len(links), q.shape[0]))
    if link_frame < 0 or link_frame >= len(links):
        raise ValueError("Link frame index link_frame must be in the range [0, N-1].")

    # Set up variables
    dof = q.shape[0]
    eps_bar = revolute_joints(links)[np.newaxis,:link_frame+1] # eps_bar = 1 if revolute, 0 if prismatic
    J_w = np.zeros((3, dof))

    # TODO: Replace the following line with your code
    T0 = T_all_to_0(links, q)
    for l in range(link_frame+1):
        z = T0[l][0:3,2] # z-axis in world frame
        J_w[:,l] = eps_bar[0, l]*z

    return J_w


def basic_jacobian(links, q, pos_in_link=None, link_frame=-1):
    """
    Computes the basic Jacobian J_0 = [J_v; J_w] for the current configuration q.

    Args:
        links   [Link, ...]: list of Link objects (see links.py)
        q           [N x 1]: Numpy array of joint values (radians for revolute joints)
        pos_in_link [3 x 1]: Numpy array for the position of the Jacobian point in link_frame
        link_frame    (int): frame index of the Jacobian point (-1 for the last link)

    Returns:
        [6 x N] Numpy array
    """
    dof = q.shape[0]
    J_0 = np.zeros((6, dof))

    # TODO: Replace the following line with your code
    J_v = linear_jacobian(links, q, pos_in_link, link_frame)
    J_w = angular_jacobian(links, q, link_frame)
    J_0 = np.vstack((J_v, J_w))

    return J_0


if __name__ == "__main__":

    # RPR Manipulator from Q1
    l1 = Link(a=0, alpha=0,        d=0,    theta=None)
    l2 = Link(a=0, alpha=-np.pi/2, d=None, theta=0)
    l3 = Link(a=0, alpha= np.pi/2, d=1,    theta=None)

    links       = [l1, l2, l3]
    q           = np.array([np.pi/2, 0.1, np.pi/2])
    pos_in_link = np.array([2, 0, 0])
    link_frame  = len(links) - 1

    J = basic_jacobian(links, q, pos_in_link, link_frame)
    print(np.round(J, 3))

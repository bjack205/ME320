#!/usr/bin/env python
"""
dynamics.py

Authors: Toki Migimatsu
         Andrey Kurenkov
         Vikranth Dwaracherla
Created: December 2017
"""

import numpy as np
from links import *
from jacobian import *

####################
# HW4 Q3: Dynamics #
####################

# TODO: Implement 2 functions below

def mass_matrix(links, q):
    """
    Computes the mass matrix for the current configuration q.

    Args:
        links [Link, ...]: list of Link objects
        q         [N x 1]: Numpy array of joint values

    Returns:
        [N x N]: Numpy array
    """
    # Check parameters
    if len(links) != q.shape[0]:
        raise ValueError("Number of links (%d) and joints q (%d) must be equal." % (len(links), q.shape[0]))

    # TODO: Replace the following line with your code
    # Variable declaration
    n = len(links)
    M = np.zeros((n, n))

    # Loop over links
    for l in range(len(links)):
        # Compute the Jacobians
        Jv = linear_jacobian(links, q, links[l].com, l)
        Jw = angular_jacobian(links, q, l)

        # Compute terms of mass matrix for one link
        A = links[l].mass*Jv.T.dot(Jv)
        B = Jw.T.dot(links[l].inertia).dot(Jw)

        # Compute the summation
        M += A + B

    return M


def gravity_vector(links, q, g=np.array([0, 0, -9.81])):
    """
    Computes the mass matrix for the current configuration q.

    Args:
        links [Link, ...]: list of Link objects
        q         [N x 1]: Numpy array of joint values
        g         [1 x 3]: Numpy array for setting gravity.

    Returns:
        [N x 1]: Numpy array
    """
    # Check parameters
    if len(links) != q.shape[0]:
        raise ValueError("Number of links (%d) and joints q (%d) must be equal." % (len(links), q.shape[0]))

    # TODO: Replace the following line with your code
    # Reshape the gravity vector so it stacks properly
    g = g.reshape((3, 1))

    # Compute the Jacobian and mg matrix for the first link
    n = len(links)
    Jv = linear_jacobian(links, q, links[0].com, 0)
    J = Jv.T
    mg = links[0].mass*g

    # Compute the Jacobian and the mg matrix for the other links and append
    for l in range(1, n):
        Jv = linear_jacobian(links, q, links[l].com, l)
        J = np.hstack((J, Jv.T))
        mg = np.vstack((mg, links[l].mass*g))

    # Compute the gravity vector
    G = -J.dot(mg)
    return G



if __name__ == "__main__":

    # Mass, center of mass, and inertia of link 1
    m1 = 1
    C1 = np.array([0, 0, 0])
    I1 = np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    ])

    # Mass, center of mass, and inertia of link 2
    m2 = 1
    C2 = np.array([0, 0, 0])
    I2 = np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    ])

    # Create likks array
    l1 = Link(a=0, alpha=0,        d=0,    theta=None, mass=m1, com=C1, inertia=I1)
    l2 = Link(a=0, alpha=-np.pi/2, d=None, theta=0,    mass=m2, com=C1, inertia=I2)
    links = [l1, l2]

    # Set robot configuration and direction of gravity
    q = np.array([0, 0.1])
    g = np.array([-9.81, 0, 0])

    # Problem 2 robot
    d1 = 2
    th2 = np.radians(90)
    d3 = 1.5
    l1 = 0.5
    l2 = 2

    m1 = 1
    C1 = np.array([0, 0, -l1])
    I1 = np.eye(3)
    m2 = 2
    C2 = np.array([l2, 0, 0])
    I2 = np.eye(3)*2
    m3 = 3
    C3 = np.array([0, 0, 0])
    I3 = np.eye(3)*3

    L1 = Link(a=0, alpha=0, d=None, theta=0, mass=m1, com=C1, inertia=I1)
    L2 = Link(a=0, alpha=-np.pi/2, d=0, theta=None, mass=m2, com=C2, inertia=I2)
    L3 = Link(a=l2, alpha=np.pi/2, d=None, theta=0, mass=m3, com=C3, inertia=I3)
    links = [L1, L2, L3]

    q = np.array([d1, th2, d3])
    g = np.array([-9.81, 0, 0])

    l = 3
    Jv = linear_jacobian(links, q, pos_in_link=links[l-1].com, link_frame=l-1)
    Jw = angular_jacobian(links, q, link_frame=l-1)
    # print(np.round(Jv, 2))


    # Compute dynamic quantities
    M = mass_matrix(links, q)
    G = gravity_vector(links, q, g)

    print(np.round(M, 2))
    print(np.round(G, 2))

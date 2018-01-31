#!/usr/bin/env python
"""
dh.py

Authors: Toki Migimatsu
         Lin Shao
         Elena Galbally Herrero
Created: December 2017
"""

import numpy as np
import rotations as r

class DH:
    """
    Entry in a DH table.
    """

    def __init__(self, a, alpha, d, theta):
        self.a     = float(a)
        self.alpha = float(alpha)
        self.d     = float(d)
        self.theta = float(theta)

    def __repr__(self):
        return "DH(a={}, alpha={}, d={}, theta={})".format(self.a, self.alpha, self.d, self.theta)



############################################
# HW2 Q2c: DH parameters to Transformation #
############################################

# TODO: Implement 1 function below

def dh_to_T(dh):
    """
    Computes the transformation matrix for the given dh row.

    Args:
        dh (DH object): ith row of the DH table

    Returns:
        T_i_to_prev [4 x 4]: Numpy array
    """
    # TODO: Replace follwing line with implementation
    Ra = BuildT(r.rot_x(dh.alpha), np.zeros((3, 1)))
    Dz = BuildT(np.eye(3), np.array([[dh.a, 0, 0]]).T)
    Rz = BuildT(r.rot_z(dh.theta), np.zeros((3, 1)))
    Dd = BuildT(np.eye(3), np.array([[0, 0, dh.d]]).T)
    return Ra.dot(Dz.dot(Rz.dot(Dd)))

def BuildT(R,D):
    T = np.hstack((R, D))
    T = np.vstack((T, np.array([[0, 0, 0, 1]])))
    return T

if __name__ == "__main__":
    """
    Sanity checks for dh.py

    Add your own sanity checks here. You may change this section however you like.
    The autograder will not run this code.
    """

    # 2c
    assert dh_to_T(DH(0.1,0.1,0.1,0.1)).shape == (4, 4)
    assert abs(np.linalg.det(dh_to_T(DH(0.1,-0.1,0.1,0.1))[0:3,0:3]) - 1) < 1e-3


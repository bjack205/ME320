# -*- coding: utf-8 -*-
"""
Created on Thu Jan 11 15:20:13 2018

@author: bjack
"""

import rotations as r
import math
import numpy as np

if __name__ == "__main__":
    # Problem 1b
    P = np.array([[2,1,1]]).T
    theta = math.radians(60)
    phi = math.radians(45)
    Pnew = np.dot(np.dot(r.rot_y(phi),r.rot_x(theta)),P)
    print(Pnew)

    # Problem 2b
    theta = math.radians(45)
    phi = math.radians(60)
    Rx = r.rot_x(theta)
    Ry = r.rot_y(phi)
    Rz = r.rot_z(theta)
    R2b = r.zyx_euler_angles_to_mat(theta, phi, theta)
    print(R2b)

    # Problem 2c
    phi = math.radians(90)
    R2c = r.zyx_euler_angles_to_mat(theta, phi, theta)
    print(np.round(R2c, 3))
    theta = 0
    R2c2 = r.zyx_euler_angles_to_mat(theta, phi, theta)
    print(np.round(R2c2, 3))

    # Problem 3
    H1 = np.zeros([4, 4])
    H1[0:3, 0:3] = np.eye(3)
    H1[:, -1] = [0, 2, 1, 1]

    H2 = np.zeros([4,4])
    H2[0:3, 0:3] = r.rot_x(math.radians(90))
    H2[3, 3] = 1
    print(H2)

    print(np.round(np.dot(H1, H2),3))

    # Problem 4
    print("\nProblem 4")
    T1 = np.array([[math.sqrt(3)/2, -1/2, 0, 1],
                   [-1/2, math.sqrt(3)/2, 0, 0],
                   [0, 0,                 1, 0],
                   [0, 0,                 0, 1]
                   ])
    T2 = np.array([[math.sqrt(3) / 2, -1 / 2, 0, -1],
                   [0, 1,                    0,   2],
                   [1/2, 0, math.sqrt(3)/2,       0],
                   [0, 0, 1, 0]
                   ])
    T3 = np.array([[1/2, math.sqrt(3)/2, 0, 1],
                   [-math.sqrt(3)/2, 1/2, 0, 2],
                   [0, 0,                1, 0],
                   [0, 0,                0, 1]
                   ])
    T4 = np.array([[1/2, math.sqrt(3)/2,  0, 1],
                   [math.sqrt(3)/2, -1/2, 0, 2],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]
                   ])
    #q1 = r.mat_to_quat(T1[0:3, 0:3])
    #q2 = r.mat_to_quat(T2[0:3, 0:3])
    print(T3[0:3, 0:3])
    q3 = r.mat_to_quat(T3[0:3, 0:3])
    #q4 = r.mat_to_quat(T4[0:3, 0:3])
    print(q3.array)
    axis,angle = r.quat_to_axis_rotation(q3)
    print("axis: %s, angle: %d" % (axis, angle))




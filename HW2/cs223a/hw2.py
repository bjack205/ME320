import rotations as r
from math import sin, cos, degrees, radians
import numpy as np
import transforms3d as t3d
import dh

# Problem 1
print("Problem 1")
theta = radians(180)
phi = radians(-60)
psi = radians(60)
R = r.rot_z(psi).dot(r.rot_y(phi).dot(r.rot_x(theta)))
R2 = r.xyz_fixed_angles_to_mat(theta, phi, psi)
print(np.round(R, 3))
print(np.allclose(R, R2))
print(r.mat_to_quat(R))

# Problem 2
print("\nProblem 2")
theta1, theta2 = radians(0), radians(90)
L1, d3 = 1, 1
a2 = 0.5
l1 = dh.DH(0, 0, L1, theta1)
l2 = dh.DH(0, radians(-90), 0, theta2)
l3 = dh.DH(-a2, radians(90), d3, np.pi)
T01 = dh.dh_to_T(l1)
T12 = dh.dh_to_T(l2)
T23 = dh.dh_to_T(l3)
T03 = T01.dot(T12.dot(T23))
print(np.round(T03, 3))

# Problem 3
print("\nProblem 3")
L1 = 1
theta1, d2, theta3, d4, theta5 = radians(0), 1, radians(0), 1, radians(0)
l1 = dh.DH(0, 0, L1, theta1)
l2 = dh.DH(0, radians(45), d2, radians(-90))
l3 = dh.DH(0, radians(90), 0, theta3 + radians(135))
l4 = dh.DH(0, radians(90), d4, radians(90))
l5 = dh.DH(0, radians(90), 0, theta5)
T01 = dh.dh_to_T(l1)
T12 = dh.dh_to_T(l2)
T23 = dh.dh_to_T(l3)
T34 = dh.dh_to_T(l4)
T45 = dh.dh_to_T(l5)
T05 = T01.dot(T12.dot(T23.dot(T34.dot(T45))))
print(np.round(T05, 3))

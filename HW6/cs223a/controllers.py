#!/usr/bin/env python
"""
main.py

Authors: Toki Migimatsu
         Mingyu Wang
         Elena Galbally-Herrera
Created: March 2018
"""

import numpy as np
from dynamics import *
from jacobian import *
from links import *


###########################
# HW6 Q1 Inverse Dynamics #
###########################

# TODO: Implement 1 function below

def inverse_dynamics(links, q, dq, gains, q_des, dq_des=None, ddq_des=None):
    """
    PD inverse dynamics (joint space dynamics) control.

        tau = M(q) ddq + G(q)

    Args:
        links [Link, ...]: List of Link objects (see links.py)
        q             [N]: Numpy array of joint values (radians for revolute joints)
        dq            [N]: Numpy array of joint velocities (rad/s for revolute joints)
        gains      (Dict): PD gains { "kp_inv_dyn": Kp, "kv_inv_dyn": Kv }
        q_des         [N]: Numpy array of desired joint values
        dq_des        [N]: Numpy array of desired joint velocities (default 0)
        ddq_des       [N]: Numpy array of desired joint accelerations (default 0)

    Returns:
        tau [N]: Numpy array of control torques
    """
    dof = len(links)
    if dq_des is None:
        dq_des = np.zeros(dof)
    if ddq_des is None:
        ddq_des = np.zeros(dof)

    # Required gains
    Kp = gains["kp_inv_dyn"]
    Kv = gains["kv_inv_dyn"]

    # TODO: Replace follwing line with implementation
    M = mass_matrix(links, q)
    G = gravity_vector(links, q)
    qdd = ddq_des - Kp*(q - q_des) - Kv*(dq - dq_des)
    tau = M.dot(qdd) + G
    return tau


############################
# HW6 Q2 Operational Space #
############################

# TODO: Implement 1 function below

def nonredundant_operational_space(links, q, dq, gains, x_des, dx_des=np.zeros(3), ddx_des=np.zeros(3), ee_offset=np.zeros(3)):
    """
    Operational Space Control (xyz position) for non-redundant manipulators.

        F = M_x(q) ddx + G_x(q)
        tau = J_v^T F

    Args:
        links [Link, ...]: List of Link objects (see links.py)
        q             [N]: Numpy array of joint values (radians for revolute joints)
        dq            [N]: Numpy array of joint velocities (rad/s for revolute joints)
        gains      (Dict): PD gains { "kp_op_space": Kp, "kv_op_space": Kv }
        x_des         [3]: Numpy array of desired end-effector position
        xq_des        [3]: Numpy array of desired end-effector velocity (default 0)
        xdq_des       [3]: Numpy array of desired end-effector acceleration (default 0)
        ee_offset     [3]: Position of end-effector in the last link frame (default 0)

    Returns:
        tau [N]: Numpy array of control torques
    """
    dof = len(links)
    if dof > 3:
        raise ValueError("nonredundant_operational_space(): len(links) cannot be greater than 3.")

    # Required gains
    Kp = gains["kp_op_space"]
    Kv = gains["kv_op_space"]

    # Mass matrix error
    delta = 0

    # TODO: Replace follwing line with implementation
    M = mass_matrix(links, q) + np.eye(3)*delta
    G = gravity_vector(links, q)
    J = linear_jacobian(links, q, ee_offset)
    
    T0 = T_all_to_0(links, q)
    H3_e = np.eye(4)
    H3_e[0:3, -1] = ee_offset
    T0e = T0[-1].dot(H3_e)
    x = T0e[0:3,-1]
    dx = J.dot(dq)
    ddx = ddx_des - Kv*(dx - dx_des) - Kp*(x - x_des)
    ddq = np.linalg.pinv(J).dot(ddx)
    tau = M.dot(ddq) + G
    return tau
    
    #############################
# HW6 Q3 Inverse Kinematics #
#############################

def solve_qp(H, f=None, A=None, b=None, A_eq=None, b_eq=None):
    """
    Solve the QP:

        min 1/2 x^T H x + f^T x
        s.t.
            A    x  <= b
            A_eq x   = b_eq

    Args:
        H        [n x n]: Symmetric positive definite matrix
        f            [n]: Numpy array of size n
        A        [m x n]: Inequality constraint matrix
        b            [m]: Inequality constraint bias
        A_eq  [m_eq x n]: Equality constraint matrix
        b_eq      [m_eq]: Equality constraint bias
    """
    import quadprog

    n = H.shape[0]
    if f is None:
        f = np.zeros(n)
    if A is None and A_eq is None:
        return np.linalg.solve(H, -f)
    if A is None:
        A = np.zeros((0, n))
        b = np.zeros(0)
    if A_eq is None:
        A_eq = np.zeros((0, n))
        b_eq = np.zeros(0)
    m_eq = b_eq.shape[0]

    C = -np.hstack((A_eq.T, A.T))
    d = -np.hstack((b_eq, b))
    return quadprog.solve_qp(H, -f, C, d, m_eq)[0]


# TODO: Implement 1 function below

def inverse_kinematics(links, q, dq, gains, x_des, ee_offset=np.zeros(3), q_lim=(None, None)):
    """
    Velocity-based inverse kinematics control.

        Desired position will be converted into a desired velocity:

            dx_des = Kp * (x_des - x) / dt

        With joint limits, we solve the quadratic program:

            min || J_v dq_des - dx_des ||^2 + alpha || dq ||^2

            s.t. dq_des >= K_lim * (q_lim_lower - q) / dt
                 dq_des <= K_lim * (q_lim_upper - q) / dt

        Without joint limits, the problem gets reduced to:

            dq_des = J_v^{+} dx_des

        For a torque-controlled robot, the velocity will be converted into a
        torque using inverse dynamics:

            q_des  = q + dq_des * dt
            tau = inverse_dynamics(q_des, dq_des)

    Args:
        links        [Link, ...]: List of Link objects (see links.py)
        q                    [N]: Numpy array of joint values (radians for revolute joints)
        dq                   [N]: Numpy array of joint velocities (rad/s for revolute joints)
        gains             (Dict): IK gains { "kp_ik": Kp, k_joint_lim": K_lim, "ik_regularizer": alpha, "dt": dt }
        x_des                [3]: Numpy array of desired end-effector position
        ee_offset            [3]: Position of end-effector in the last link frame (default 0)
        q_lim (q_lower, q_upper): Tuple of lower and upper joint limits (default (None, None))

    Returns:
        tau [N]: Numpy array of control torques
    """
    dof = len(links)

    # Required gains
    Kp    = gains["kp_inv_kin"]
    K_lim = gains["k_joint_lim"]
    alpha = gains["ik_regularizer"]
    dt    = gains["dt"]

    # TODO: Replace follwing line with implementation
    J = linear_jacobian(links, q, ee_offset)

    T0 = T_all_to_0(links, q)
    H3_e = np.eye(4)
    H3_e[0:3, -1] = ee_offset
    T0e = T0[-1].dot(H3_e)
    x = T0e[0:3, -1]

    v = Kp * (x_des - x) / dt

    A = []
    b = []


    for j in range(dof):
        if q_lim[1] is not None:
            A.append(one_hot(j, dof).T)
            b.append(q_lim[1][j] - q[j])
        if q_lim[0] is not None:
            temp = np.zeros((dof,))
            temp[j] = -1
            A.append(v)
            b.append(q[j] - q_lim[0][j])
    if not A:
        # Solve using pseudoinverse
        dq_des = np.linalg.pinv(J).dot(v)
    else:
        A = np.array(A)
        b = np.array(b)
        b *= K_lim / dt
        H = (J.T).dot(J)+alpha*2
        f = -((v.T).dot(J)).T
        dq_des = solve_qp(H, f, A, b)

    # print(A)
    # print(b)
    # print(H)
    # print(f)
    # print(v)
    
    q_des = q + dq_des * dt

    tau = inverse_dynamics(links, q, dq, gains, q_des, dq_des)
    return tau


def one_hot(n, k):
    v = np.zeros((k,))
    v[n] = 1
    return v

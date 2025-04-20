#!/usr/bin/env python3

import numpy as np

def dh_transform(a, alpha, d, theta):
    """
    Create the standard DH homogeneous transform.
    Parameters:
    - a     : link length
    - alpha : link
    - d     : joint length
    - theta : joint angel
    function to return homogenus transform for signle DH row
    """
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),                np.cos(alpha),               d],
        [0,              0,                            0,                           1]
    ])

def forward_kinematics(thetas, lengths):
    """
    Parameters:
    - thetas : list or array of 4 joint angles [θ1, θ2, θ3, θ4] in radians
    - lengths: list or array of 4 link lengths [L0, L1, L2, L3]
    Returns:
    - pos:   (x, y, z) position of the end-effector in the base frame
    - T_end: full 4×4 homogeneous transform from base to end-effector
    """
    #DH parameters
    a     = [lengths[0], lengths[1], lengths[2], lengths[3]]
    alpha = [ np.pi/2,   -np.pi/2,    np.pi/2,    0      ]
    d     = [0, 0, 0, 0]

    T = np.eye(4)
    #accumulate transforms
    for i in range(4):
        A_i = dh_transform(a[i], alpha[i], d[i], thetas[i])
        T = T @ A_i

    pos = T[0:3, 3]
    return pos, T

if __name__ == "__main__":
    import math

   
    lengths = [1.0, 1,1, 1]  #modify as needed

 
    degs = [90, 90, 90, 00]        #modify as needed
    thetas = [math.radians(d) for d in degs]

    pos, T_end = forward_kinematics(thetas, lengths)
    print("End-effector position (x, y, z):", pos)
    print("\nFull transform T_0^4 =\n", T_end)


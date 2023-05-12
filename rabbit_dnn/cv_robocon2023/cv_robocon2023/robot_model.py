import numpy as np

a1 = np.pi/4
a2 = 3*np.pi/4
a3 = 5*np.pi/4
a4 = 7*np.pi/4

def forward_kinematic(u1, u2, u3, u4, theta):

    rot_mat = np.array([
        [np.cos(theta), np.sin(theta), 0],
        [-np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])

    J_for = np.array([
        [np.sin(a1), -np.sin(a2), np.sin(a3), -np.sin(a4)],
        [np.cos(a1), -np.cos(a2), np.cos(a3), -np.cos(a4)],
        [1/(2*0.22), 1/(2*0.22), 1/(2*0.22), 1/(2*0.22)]
    ], dtype=np.float32)

    for_vec = rot_mat.T@J_for@np.array([u1, u2, u3, u4])

    return for_vec


def inverse_kinematic(vx, vy, vth, theta):

    rot_mat = np.array([
        [np.cos(theta), np.sin(theta), 0],
        [-np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])

    J_inv = np.array([
        [np.sin(a1), -np.sin(a2), np.sin(a3), -np.sin(a4)],
        [np.cos(a1), -np.cos(a2), np.cos(a3), -np.cos(a4)],
        [0.22, 0.22, 0.22, 0.22]
    ], dtype=np.float32).T

    inv_vec = J_inv@rot_mat@np.array([vx, vy, vth])

    return inv_vec



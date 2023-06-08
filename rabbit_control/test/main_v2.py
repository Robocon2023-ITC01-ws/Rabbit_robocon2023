import numpy as np

import casadi as ca
from nmpc_solver import NMPCSolver
from rabbit_omni import RabbitModel



model = RabbitModel()


mpciter = 0
N = 10
step_horizon = 0.1
sim_time = 10
t0 = 0

current_states = np.array([0, 0, 0], dtype=np.float64)
current_controls = np.array([0, 0, 0, 0], dtype=np.float64)


states = np.tile(current_states.reshape(3, 1), N+1)
controls = np.tile(current_controls.reshape(4, 1), N)

next_trajectories = np.tile(current_states.reshape(3, 1), N+1)
next_controls = np.tile(current_controls.reshape(4, 1), N)


initX = [0.0, 0.0, 0.0]
initU = [0.0, 0.0, 0.0, 0.0]
# lowX =  [-5.0, -5.0,  -np.pi]
# highX = [5.0, 5.0,  np.pi]
lowX = [-ca.inf, -ca.inf, -ca.inf]
highX = [ca.inf, ca.inf, ca.inf]
lowU = -30
highU = 30
# lowU = [-30, -20, -20, -20]
# highU = [20, 20, 20, 20]
mat_Q = [750, 750, 2000]
mat_R = [1, 1, 1, 1]

controller = NMPCSolver(
    lowX=lowX, highX=highX,
    lowU=lowU, highU=highU,
    Q=mat_Q, R=mat_R,
    N=N, dt=step_horizon
)

f, solver, args = controller.nmpc_setup()


while (mpciter * step_horizon < sim_time):

    args['p'] = np.concatenate([
        next_trajectories.T.reshape(-1, 1),
        next_controls.T.reshape(-1, 1)
    ])

    args['x0'] = np.concatenate([
        states.T.reshape(-1, 1),
        controls.T.reshape(-1, 1)
    ])

    sol = solver(
        x0=args['x0'],
        p = args['p'],
        lbx=args['lbx'],
        ubx=args['ubx'],
        lbg=args['lbg'],
        ubg=args['ubg'],
    )

    sol_x = ca.reshape(sol['x'][:3*(N+1)], 3, N+1)
    sol_u = ca.reshape(sol['x'][3*(N+1):], 4, N)

    u1 = sol_u.full()[0, 0]
    u2 = sol_u.full()[1, 0]
    u3 = sol_u.full()[2, 0]
    u4 = sol_u.full()[3, 0]
    theta = sol_x.full()[2, 0]


    for j in range(N):
        t0 = step_horizon * j + mpciter
        next_trajectories[0, 0] = current_states[0]
        next_trajectories[1, 0] = current_states[1]
        next_trajectories[2, 0] = current_states[2]

        next_trajectories[:, j+1] = np.array([
            10,
            10,
            0.0
        ])
        if np.linalg.norm(current_states-next_trajectories[:, -1], 2) > 0.5:
            next_controls = np.tile(np.array([30, 30, 30, 30], dtype=np.float64), N)
        elif np.linalg.norm(current_states-next_trajectories[:, -1], 2) < 0.5:
            next_controls = np.tile(np.array([0, 0, 0, 0], dtype=np.float64), N)
    x_next = current_states + step_horizon * model.forward_kinematic_tran(u1, u2, u3, u4, theta, "numpy")

    current_states = x_next

    u1 = np.round(u1, 3)
    u2 = np.round(u2, 3)
    u3 = np.round(u3, 3)
    u4 = np.round(u4, 3)
    current_controls = np.array([u1, u2, u3, u4])

    states = np.tile(current_states.reshape(3, 1), N+1)
    controls = np.tile(current_controls.reshape(4, 1), N)

    # print(current_states)

    # print(u1, u2, u3, u4)
    print(np.round(model.forward_kinematic_tran(u1, u2, u3, u4, theta, "numpy"), 3))

    mpciter += 1
    

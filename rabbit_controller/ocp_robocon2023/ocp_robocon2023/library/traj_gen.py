import numpy as np
import matplotlib.pyplot as plt
import casadi as ca

from bezier_path import calc_4points_bezier_path

r = 0.06
R = 0.22

a1 = np.pi/4
a2 = 3*np.pi/4
a3 = 5*np.pi/4
a4 = 7*np.pi/4

N = 20
step_time = 0.1
mpciter = 0
sim_time = 30

x_min = -10
y_min = -10
theta_min = -np.pi

x_max = 10
y_max = 10
theta_max = np.pi

u_min = -33.33
u_max =  33.3

start_x = 2
start_y = 2
start_yaw = 1.57

end_x = 0
end_y = 0
end_yaw = 0

offset = 5.0


def forward_kinematic(u1, u2, u3, u4, theta):
    rot_mat = np.array([
        [np.cos(theta), np.sin(theta), 0],
        [-np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])

    J_for = (r/2)*np.array([
        [np.sin(a1), -np.sin(a2), np.sin(a3), -np.sin(a4)],
        [np.cos(a1), -np.cos(a2), np.cos(a3), -np.cos(a4)],
        [1/(2*0.22), 1/(2*0.22), 1/(2*0.22), 1/(2*0.22)]
    ])

    for_vec = rot_mat.T@J_for@np.array([u1, u2, u3, u4])

    return for_vec

def plot_arrow(x, y, yaw, length=0.05, width=0.3, fc="b", ec="k"):
    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * np.cos(yaw), length * np.sin(yaw),
            fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)

# States
x = ca.SX.sym('x')
y = ca.SX.sym('y')
theta = ca.SX.sym('theta')
states = ca.vertcat(x, y, theta)
n_states = states.numel()
u1 = ca.SX.sym('u1')
u2 = ca.SX.sym('u2')
u3 = ca.SX.sym('u3')
u4 = ca.SX.sym('u4')
controls = ca.vertcat(u1, u2, u3, u4)
n_controls = controls.numel()

X = ca.SX.sym('X', n_states, N+1)
X_ref = ca.SX.sym('X_ref', n_states, N+1)
U = ca.SX.sym('U', n_controls, N)
U_ref = ca.SX.sym('U_ref', n_controls, N)

Q = np.diag([750, 750, 2000])
R = np.diag([1, 1, 1, 1])

cost_fn = 0.0
g = X[:, 0] - X_ref[:, 0]


rot_mat = ca.vertcat(
    ca.horzcat(ca.cos(theta), ca.sin(theta), 0),
    ca.horzcat(-ca.sin(theta), ca.cos(theta), 0),
    ca.horzcat(0, 0, 1)
)

J_for = (r/2)*ca.vertcat(
    ca.horzcat(ca.sin(a1), -ca.sin(2), ca.sin(a3), -ca.sin(a4)),
    ca.horzcat(ca.cos(a1), -ca.cos(a2), ca.cos(a3), -ca.cos(a4)),
    ca.horzcat(1/(2*0.22), 1/(2*0.22), 1/(2*0.22), 1/(2*0.22))
)

rhs = rot_mat.T@J_for@ca.vertcat(u1, u2, u3, u4)

f = ca.Function('f', [states, controls], [rhs])

for k in range(N):
    st_err = X[:, k] - X_ref[:, k]
    con_err = U[:, k] - U_ref[:, k]
    cost_fn = cost_fn + st_err.T@Q@st_err + con_err.T@R@con_err
    st_next = X[:, k+1]
    st_euler = X[:, k] + step_time * f(X[:, k], U[:, k])
    g = ca.vertcat(g, st_next-st_euler)

cost_fn = cost_fn + (X[:, N]-X_ref[:, N]).T@Q@(X[:, N]-X_ref[:, N])

opt_dec = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
opt_par = ca.vertcat(ca.reshape(X_ref, -1, 1), ca.reshape(U_ref, -1, 1))


nlp_probs = {
    'f': cost_fn,
    'x': opt_dec,
    'p': opt_par,
    'g': g
}

nlp_opts = {
    'ipopt.max_iter': 5000,
    'ipopt.print_level': 0,
    'ipopt.acceptable_tol': 1e-6,
    'ipopt.acceptable_obj_change_tol': 1e-4,
    'print_time': 0}

solver = ca.nlpsol('solver', 'ipopt', nlp_probs, nlp_opts)

lbx = ca.DM.zeros((n_states*(N+1)+n_controls*N, 1))
ubx = ca.DM.zeros((n_states*(N+1)+n_controls*N, 1))

lbx[0: n_states*(N+1): n_states] = x_min
lbx[1: n_states*(N+1): n_states] = y_min
lbx[2: n_states*(N+1): n_states] = theta_min

ubx[0: n_states*(N+1): n_states] = x_max
ubx[1: n_states*(N+1): n_states] = y_max
ubx[2: n_states*(N+1): n_states] = theta_max

lbx[n_states*(N+1) : n_states*(N+1)+n_controls*N: n_controls] = u_min
lbx[n_states*(N+1)+1: n_states*(N+1)+n_controls*N: n_controls] = u_min
lbx[n_states*(N+1)+2: n_states*(N+1)+n_controls*N: n_controls] = u_min
lbx[n_states*(N+1)+3: n_states*(N+1)+n_controls*N: n_controls] = u_min

ubx[n_states*(N+1) : n_states*(N+1)+n_controls*N: n_controls] = u_max
ubx[n_states*(N+1)+1: n_states*(N+1)+n_controls*N: n_controls] = u_max
ubx[n_states*(N+1)+2: n_states*(N+1)+n_controls*N: n_controls] = u_max
ubx[n_states*(N+1)+3: n_states*(N+1)+n_controls*N: n_controls] = u_max

args = {
    'lbg': ca.DM.zeros((n_states*(N+1), 1)),
    'ubg': ca.DM.zeros((n_states*(N+1), 1)),
    'lbx': lbx,
    'ubx': ubx
}

# path, _ = calc_4points_bezier_path(
#     start_x, start_y, start_yaw,
#     end_x, end_y, end_yaw,
#     offset
# )



path1, _ = calc_4points_bezier_path(
    0, 0, -1.57,
    5, -5, 0.0,
    1.0
)

goal1 = np.vstack([path1[:, 0], path1[:, 1], np.append(np.arctan2(np.diff(path1[:, 1]), np.diff(path1[:, 0])), 0.0)])


path2, _ = calc_4points_bezier_path(
    2, 2, 1.57,
    0, 0, 0.0,
    3.0
)

goal2 = np.vstack([path2[:, 0], path2[:, 1],  np.tile(1.57, 100)])

path3, _ = calc_4points_bezier_path(
    5.5, -3.0, 1.57,
    3.5, -3.0, 1.57,
    10.0
)

goal3 = np.vstack([path3[:, 0], path3[:, 1], np.tile(1.57, 100)])

# path4, _ = calc_4points_bezier_path(
#     3.5, -3.0, 1.57,
#     3.5, 0.0, 0.0,
#     10.0
# )

# goal4 = np.vstack([path4[:, 0], path4[:, 1], np.append(np.arctan2(np.diff(path4[:, 0]), np.diff(path4[:, 1])), 0.0)])


goal_states = np.hstack([goal1])

ax = goal_states[0, :]
ay = goal_states[1, :]

current_states = np.array([0.0, 0.0, -1.57], dtype=np.float64)
current_controls = np.array([0, 0, 0, 0], dtype=np.float64)

states = np.tile(current_states.reshape(3, 1), N+1)
controls = np.tile(current_controls.reshape(4, 1), N)

next_trajectories = np.tile(current_states.reshape(3, 1), N+1)
next_controls = np.tile(current_controls.reshape(4, 1), N)


if __name__ == "__main__":

    while (mpciter * step_time < sim_time):

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

        sol_x = ca.reshape(sol['x'][:n_states*(N+1)], 3, N+1)
        sol_u = ca.reshape(sol['x'][n_states*(N+1):], 4, N)

        u1 = sol_u.full()[0, 0]
        u2 = sol_u.full()[1, 0]
        u3 = sol_u.full()[2, 0]
        u4 = sol_u.full()[3, 0]

        theta = sol_x.full()[2, 0]

        for j in range(N):
            index = mpciter + j
            if index >= goal_states.shape[1]:
                index = goal_states.shape[1]-1
            next_trajectories[0, 0] = current_states[0]
            next_trajectories[1, 0] = current_states[1]
            next_trajectories[2, 0] = current_states[2]
            next_trajectories[:, j+1] = goal_states[:, index]
            # next_trajectories[0, 1] = 2
            next_controls = np.tile(np.array([30, 30, 30, 30]).reshape(4, 1), N)

        # x_next = current_states + forward_kinematic(u1, u2, u3, u4, theta) * step_time

        x_next = ca.DM.full(current_states.reshape(3, 1) + step_time * f(current_states, current_controls))

        current_states = x_next

        current_controls = np.array([u1, u2, u3, u4])

        states = np.tile(current_states.reshape(3, 1), N+1)
        controls = np.tile(current_controls.reshape(4, 1), N)

        plt.clf()
        plt.plot(ax, ay, "b")
        plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
        plot_arrow(current_states[0], current_states[1], current_states[2])
        plt.plot(ax, ay, marker="x", color="blue", label="Input Trajectory")
        plt.scatter(sol_x.full()[0, :], sol_x.full()[1, :], marker="*", color="red", label="Predicted value")
        plt.plot(current_states[0], current_states[1], marker="*", color="black")
        plt.axis("equal")
        plt.grid(True)
        plt.legend()
        plt.pause(0.0001)

        print(forward_kinematic(u1, u2, u3, u4, theta))
        u1 = round(u1, 3)
        u2 = round(u2, 3)
        u3 = round(u3, 3)
        u4 = round(u4, 3)
        # print(u1, u2, u3, u4)

        mpciter =+ mpciter +1
        # print(mpciter)

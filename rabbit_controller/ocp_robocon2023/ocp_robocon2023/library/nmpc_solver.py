import numpy as np
import casadi as ca

from .rabbit_omni import RabbitModel


class NMPCSolver:

    def __init__(self, lowX, highX, lowU, highU, Q, R, N, dt):

        self.lowX = lowX
        self.highX = highX
        self.lowU = lowU
        self.highU = highU
        
        self.Q = Q
        self.R = R

        self.N = N
        self.dt = dt

        self.rabbit_model = RabbitModel()

    
    def nmpc_setup(self):
        # States
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        states = ca.vertcat(x, y, theta)
        n_states = states.numel()
        # Controls
        u1 = ca.SX.sym('u1')
        u2 = ca.SX.sym('u2')
        u3 = ca.SX.sym('u3')
        u4 = ca.SX.sym('u4')
        controls = ca.vertcat(u1, u2, u3, u4)
        n_controls = controls.numel()


        # Symbolics Matrix
        X = ca.SX.sym('X', n_states, self.N+1)
        X_ref = ca.SX.sym('X_ref', n_states, self.N+1)
        U = ca.SX.sym('U', n_controls, self.N)
        U_ref = ca.SX.sym('U_re', n_controls, self.N)

        # Tuning Matrix

        Q = np.diag(self.Q)
        R = np.diag(self.R)

        # Cost
        cost_fn = 0.0
        g = X[:, 0] - X_ref[:, 0]

        # rot_mat = ca.vertcat(
        #     ca.horzcat(ca.cos(theta), ca.sin(theta), 0),
        #     ca.horzcat(-ca.sin(theta), ca.cos(theta), 0),
        #     ca.horzcat(0, 0, 1)
        # )

        # J = (0.06/2)*ca.vertcat(
        #     ca.horzcat(ca.sin(ca.pi/4), -ca.sin(3*ca.pi/4), ca.sin(5*ca.pi/4), -ca.sin(7*ca.pi/4)),
        #     ca.horzcat(ca.cos(ca.pi/4), -ca.cos(3*ca.pi/4), ca.cos(5*ca.pi/4), -ca.cos(7*ca.pi/4)),
        #     ca.horzcat(1/(2*0.22), 1/(2*0.22),1/(2*0.22), 1/(2*0.22))
        # )

        # rhs = rot_mat.T@J@controls
        
        rhs = self.rabbit_model.forward_kinematic_tran(u1, u2, u3, u4, theta, "sym")

        f = ca.Function('f', [states, controls], [rhs])

        for k in range(self.N):
            st_err = X[:, k] - X_ref[:, k]
            con_err = U[:, k] - U_ref[:, k]
            cost_fn = cost_fn + st_err.T@Q@st_err + con_err.T@R@con_err
            st_next = X[:, k+1]
            st_euler = X[:, k] + self.dt*f(X[:,k], U[:, k])
            g = ca.vertcat(g, st_next-st_euler)

        cost_fn = cost_fn + (X[:, self.N]-X_ref[:, self.N]).T@Q@(X[:, self.N]-X_ref[:, self.N])

        opt_var = ca.vertcat(
            ca.reshape(X, -1, 1),
            ca.reshape(U, -1, 1)
        )

        opt_par = ca.vertcat(
            ca.reshape(X_ref, -1, 1),
            ca.reshape(U_ref, -1, 1)
        )

        nlp_prob = {
            'f': cost_fn,
            'x': opt_var,
            'p': opt_par,
            'g': g
        }

        nlp_opts = {
                    # 'calc_lam_p': True,
                    # 'calc_lam_x': True,
                    # 'mu_min': 0.1,
                    # 'warm_start_init_point': 'yes',
                    'verbose': False,
                    'ipopt.tol': 1e-4,
                    'ipopt.warm_start_init_point': "yes",
                    'ipopt.max_iter': 5000,
                    'ipopt.print_level': 0,
                    'ipopt.acceptable_tol': 1e-6,
                    'ipopt.acceptable_obj_change_tol': 1e-4,
                    'print_time': 0}
        
        solver = ca.nlpsol('solver', 'ipopt', nlp_prob, nlp_opts)

        lbx = ca.DM.zeros((n_states*(self.N+1) + n_controls*self.N, 1))
        ubx = ca.DM.zeros((n_states*(self.N+1) + n_controls*self.N, 1))

        lbx[0: n_states*(self.N+1): n_states] = self.lowX[0]
        lbx[1: n_states*(self.N+1): n_states] = self.lowX[1]
        lbx[2: n_states*(self.N+1): n_states] = self.lowX[2]


        ubx[0: n_states*(self.N+1): n_states] = self.highX[0]
        ubx[1: n_states*(self.N+1): n_states] = self.highX[1]
        ubx[2: n_states*(self.N+1): n_states] = self.highX[2]

        lbx[n_states*(self.N+1)  : n_states*(self.N+1)+n_controls*self.N: 4] = self.lowU
        lbx[n_states*(self.N+1)+1: n_states*(self.N+1)+n_controls*self.N: 4] = self.lowU
        lbx[n_states*(self.N+1)+2: n_states*(self.N+1)+n_controls*self.N: 4] = self.lowU
        lbx[n_states*(self.N+1)+3: n_states*(self.N+1)+n_controls*self.N: 4] = self.lowU


        ubx[n_states*(self.N+1)  : n_states*(self.N+1)+n_controls*self.N: 4] = self.highU
        ubx[n_states*(self.N+1)+1: n_states*(self.N+1)+n_controls*self.N: 4] = self.highU
        ubx[n_states*(self.N+1)+2: n_states*(self.N+1)+n_controls*self.N: 4] = self.highU
        ubx[n_states*(self.N+1)+3: n_states*(self.N+1)+n_controls*self.N: 4] = self.highU


        args = {
                'lbg': ca.DM.zeros((n_states*(self.N+1), 1)),
                'ubg': ca.DM.zeros((n_states*(self.N+1), 1)),
                'lbx': lbx,
                'ubx': ubx
            }

        return f, solver, args

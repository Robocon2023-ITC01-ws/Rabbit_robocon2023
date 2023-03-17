import numpy as np
import casadi as ca

from bezier_path import calc_4points_bezier_path


class CasadiDIff:

    def __init__(self,
                lowX, highX,
                lowU, highU,
                mat_Q, mat_R,
                N_horizons, sampling_time
    ):
        # NMPC params
        self.lowX = lowX
        self.highX = highX
        self.lowU = lowU
        self.highU = highX

        self.mat_Q = mat_Q
        self.mat_R = mat_R

        self.N = N_horizons
        self.dt = sampling_time


    def casadi_solver(self):
        ## States
        x = ca.SX.sym("x")
        y = ca.SX.sym("y")
        theta = ca.SX.sym("theta")
        states = ca.vertcat(x, y, theta)
        num_states = states.numel()
        ## Controls
        v = ca.SX.sym("v")
        omega = ca.SX.sym("omega")
        controls = ca.vertcat(v, omega)
        num_controls = controls.numel()
        ## Matrix
        X = ca.SX.sym("X", num_states, self.N+1)
        X_ref = ca.SX.sym("X_ref", num_states, self.N+1)
        U = ca.SX.sym("U", num_controls, self.N)
        U_ref = ca.SX.sym("U_ref", num_controls, self.N)

        Q = np.diag(self.mat_Q)
        R = np.diag(self.mat_R)

        # Cost
        cost_fn = 0.0
        g = X[:, 0] - X_ref[:, 0]

        # Nonlinear function
        rhs = ca.vertcat(
            v*ca.cos(theta),
            v*ca.sin(theta),
            omega
        )

        f = ca.Function('f', [states, controls], [rhs])

        ## Stage cost
        for k in range(self.N):
            st_err = X[:, k] - X_ref[:, k]
            con_err = U[:, k] - U_ref[:, k]
            cost_fn = cost_fn + st_err.T@Q@st_err + con_err.T@R@con_err
            st_next = X[:, k+1]
            st_next_euler = X[:, k] + self.dt * f(X[:, k], U[:, k])
            g = ca.vertcat(g, st_next-st_next_euler)

        ## Terminal cost
        cost_fn = cost_fn + X[:, self.N].T@Q@X[:, self.N]

        optimal_var = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
        optimal_par = ca.vertcat(ca.reshape(X_ref, -1, 1), ca.reshape(U_ref, -1, 1))


        ## Nonlinear problem
        nlp_prob = {
            'f': cost_fn,
            'x': optimal_var,
            'p': optimal_par,
            'g': g
        }

        nlp_opts = {
                'ipopt.max_iter': 5000,
                'ipopt.print_level': 0,
                'ipopt.acceptable_tol': 1e-6,
                'ipopt.acceptable_obj_change_tol': 1e-4,
                'print_time': 0}
        solver = ca.nlpsol('solver', 'ipopt', nlp_prob, nlp_opts)

        lbx = ca.DM.zeros((num_states*(self.N+1) + num_controls*self.N, 1))
        ubx = ca.DM.zeros((num_states*(self.N+1) + num_controls*self.N, 1))

        lbx[0: num_states*(self.N+1): num_states] = self.lowX[0]
        lbx[1: num_states*(self.N+1): num_states] = self.lowX[1]
        lbx[2: num_states*(self.N+1): num_states] = self.lowX[2]


        ubx[0: num_states*(self.N+1): num_states] = self.highX[0]
        ubx[1: num_states*(self.N+1): num_states] = self.highX[1]
        ubx[2: num_states*(self.N+1): num_states] = self.highX[2]

        lbx[num_states*(self.N+1): num_states*(self.N+1)+num_controls*self.N: 4] = self.lowU[0]
        lbx[num_states*(self.N+1)+1: num_states*(self.N+1)+num_controls*self.N: 4] = self.lowU[1]
        lbx[num_states*(self.N+1)+2: num_states*(self.N+1)+num_controls*self.N: 4] = self.lowU[2]
        lbx[num_states*(self.N+1)+3: num_states*(self.N+1)+num_controls*self.N: 4] = self.lowU[3]


        ubx[num_states*(self.N+1): num_states*(self.N+1)+num_controls*self.N: 4] = self.highU[0]
        ubx[num_states*(self.N+1)+1: num_states*(self.N+1)+num_controls*self.N: 4] = self.highU[1]
        ubx[num_states*(self.N+1)+2: num_states*(self.N+1)+num_controls*self.N: 4] = self.highU[2]
        ubx[num_states*(self.N+1)+3: num_states*(self.N+1)+num_controls*self.N: 4] = self.highU[3]

        args = {
                'lbg': ca.DM.zeros((num_states*(self.N+1), 1)),
                'ubg': ca.DM.zeros((num_states*(self.N+1), 1)),
                'lbx': lbx,
                'ubx': ubx
            }

        return f, solver, args

    def main(self):

        # Simulation
        t0 = 0
        mpciter = 0
        init_states = np.array([0.0, 0.0, 0.0]).reshape(1, -1)
        current_states = init_states.copy()
        init_control = np.array([0.0, 0.0])
        state = np.tile(current_states.reshape(-1, 1), self.N+1).T
        control = np.tile(init_control.reshape(-1, 1), self.N).T
        next_trajectories = state.copy()
        next_controls = control.copy()

        f, solver, args = self.casadi_solver()
        while (mpciter * self.dt < 10):
            args['p'] = np.concatenate([
                next_trajectories.reshape(-1, 1),
                next_controls.reshape(-1, 1)
            ])
            args['x0'] = np.concatenate([

            ])
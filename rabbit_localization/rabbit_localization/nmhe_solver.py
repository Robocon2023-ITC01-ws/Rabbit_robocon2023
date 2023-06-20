import numpy as np
import casadi as ca

from rabbit_localization.rabbit_meca import RabbitModel

class NMHESolver:

    def __init__(self, lowX, highX, lowU, highU, V, W, N_mhe):

        self.lowX = lowX
        self.highX = highX

        self.lowU = lowU
        self.highU = highU

        self.V = V
        self.W = W

        self.N_mhe = N_mhe

        self.r = 0.0475

        self.rabbit_model = RabbitModel()


    def nmhe_solver(self):
        ### STATES ###
        x = ca.SX.sym("x")
        y = ca.SX.sym("y")
        yaw = ca.SX.sym("yaw")
        states = ca.vertcat(x, y, yaw)
        n_states = states.numel()
        ### WHeel model
        w1 = ca.SX.sym('w1')
        w2 = ca.SX.sym('w2')
        w3 = ca.SX.sym('w3')
        w4 = ca.SX.sym('w4')
        controls = ca.vertcat(w1, w2 ,w3, w4)
        n_controls = controls.numel()
        ### Rotary ###
        u1 = ca.SX.sym("u1")
        u2 = ca.SX.sym("u2")
        u3 = ca.SX.sym("u3")
        inputs = ca.vertcat(u1, u2, u3)
        n_inputs = inputs.numel()

        X = ca.SX.sym("X", n_states, self.N_mhe+1)
        X_meas = ca.SX.sym("X_meas", n_states, self.N_mhe+1)
        U = ca.SX.sym("U", n_controls, self.N_mhe)
        U_meas = ca.SX.sym("U_meas", n_controls, self.N_mhe)

        U_rot = ca.SX.sym('U_rot', n_inputs, self.N_mhe)

        rhs = self.rabbit_model.forward_kinematic(w1, w2, w3, w4, yaw, "sym")
        
        rotary_model = ca.vertcat(
            u1*ca.cos(yaw)-u2*ca.sin(yaw),
            u1*ca.sin(yaw)+u2*ca.cos(yaw),
            u3
        )

        f = ca.Function('f', [states, controls], [rhs])

        # h = ca.Function('h', [states], [])

        cost_fn = 0.0
        g = X[:, 0]-X_meas[:, 0]

        for k in range(self.N_mhe):
            st = X[:, k]
            h_x = X_meas[:, k]
            st_err = st-h_x
            cost_fn = cost_fn + st_err.T@self.V@st_err
        
        for k in range(self.N_mhe):
            con = U[:, k]
            con_meas = U_meas[:, k]
            con_err = con-con_meas
            cost_fn = cost_fn + con_err.T@self.W@con_err

        for k in range(self.N_mhe):
            st_next = X[:, k+1]
            st_euler = X[:, k] + f(X[:, k], U[:, k])
            g = ca.vertcat(st_next-st_euler)


        exact_x = ca.vertcat(
            ca.reshape(X, -1, 1),
            ca.reshape(U, -1, 1)
        )
        meas_x = ca.vertcat(
            ca.reshape(X_meas, -1, 1),
            ca.reshape(U_meas, -1, 1)
        )

        nlp_prob = {
            'f': cost_fn,
            'x': exact_x,
            'p': meas_x,
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

        lbx = ca.DM.zeros((n_states*(self.N_mhe+1) + n_controls*self.N_mhe, 1))
        ubx = ca.DM.zeros((n_states*(self.N_mhe+1) + n_controls*self.N_mhe, 1))

        lbx[0: n_states*(self.N_mhe+1): n_states] = self.lowX[0]
        lbx[1: n_states*(self.N_mhe+1): n_states] = self.lowX[1]
        lbx[2: n_states*(self.N_mhe+1): n_states] = self.lowX[2]

        ubx[0: n_states*(self.N_mhe+1): n_states] = self.highX[0]
        ubx[1: n_states*(self.N_mhe+1): n_states] = self.highX[1]
        ubx[2: n_states*(self.N_mhe+1): n_states] = self.highX[2]

        lbx[n_states*(self.N_mhe+1)  : n_states*(self.N_mhe+1)+n_controls*self.N_mhe: 4] = self.lowU
        lbx[n_states*(self.N_mhe+1)+1: n_states*(self.N_mhe+1)+n_controls*self.N_mhe: 4] = self.lowU
        lbx[n_states*(self.N_mhe+1)+2: n_states*(self.N_mhe+1)+n_controls*self.N_mhe: 4] = self.lowU
        lbx[n_states*(self.N_mhe+1)+3: n_states*(self.N_mhe+1)+n_controls*self.N_mhe: 4] = self.lowU


        ubx[n_states*(self.N_mhe+1)  : n_states*(self.N_mhe+1)+n_controls*self.N_mhe: 4] = self.highU
        ubx[n_states*(self.N_mhe+1)+1: n_states*(self.N_mhe+1)+n_controls*self.N_mhe: 4] = self.highU
        ubx[n_states*(self.N_mhe+1)+2: n_states*(self.N_mhe+1)+n_controls*self.N_mhe: 4] = self.highU
        ubx[n_states*(self.N_mhe+1)+3: n_states*(self.N_mhe+1)+n_controls*self.N_mhe: 4] = self.highU



        args = {
                'lbg': ca.DM.zeros((n_states*(self.N_mhe+1), 1)),
                'ubg': ca.DM.zeros((n_states*(self.N_mhe+1), 1)),
                'lbx': lbx,
                'ubx': ubx
            }
        
        return f, solver, args
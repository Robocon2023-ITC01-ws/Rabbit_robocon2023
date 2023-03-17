import numpy as np
import casadi as ca
import math

from .rabbit_robot import RabbitModel

class CasadiNMPC:

    def __init__(self,
        initX, initU,
        lowX, highX,
        lowU, highU,
        mat_Q, mat_R,
        N, dt, sim_time
    ):
        super(CasadiNMPC, self).__init__()
        #Model
        self.rabbit_model = RabbitModel()

        self.initX = initX
        self.initU = initU
        self.lowX = lowX
        self.highX = highX
        self.lowU = lowU
        self.highU = highU
        self.mat_Q = mat_Q
        self.mat_R = mat_R

        self.N = N
        self.dt = dt
        self.sim_time = sim_time
        self.index = 0

        self.rot_3d_z = lambda theta: ca.vertcat(
            ca.horzcat(ca.cos(theta), ca.sin(theta), 0),
            ca.horzcat(-ca.sin(theta),  ca.cos(theta), 0),
            ca.horzcat(         0,           0, 1)
        )

    # Shift timestep at each iteration
    def shift_timestep(self, step_horizon, t0, x0, x_f, u, f):
        x0 = x0.reshape((-1,1))
        t = t0 + step_horizon
        f_value = f(x0, u[:, 0])
        st = ca.DM.full(x0 + (step_horizon) * f_value)
        # st = np.array([0, 0, 0])
        u = np.concatenate((u[:, 1:], u[:, -1:]), axis=1)
        x_f = np.concatenate((x_f[:, 1:], x_f[:, -1:]), axis=1)
        return t, st, x_f, u

    # Function to calculate reference trajectory
    def reference_state_and_control(self, t, step_horizon, x0, N, type="None"):
        # initial state
        x = x0.reshape(1, -1).tolist()[0]
        u = []
        # Speed drop
        Tf = 3 # delay seconds
        speed_exp_up = lambda t: Tf*(10-10*np.exp(-2*t))
        speed_exp_down = lambda t: Tf*(10-10*np.exp(-2*(t-30)))
        # set points for calculating reference control
        x_ref, y_ref, theta_ref = [], [], []
        v1_ref, v2_ref, v3_ref, v4_ref = [], [], [], []
        x_ref_, y_ref_, theta_ref_ = 0, 0, 0
        vx_r, vy_r, vth_r = 0, 0, 0
        # state for N predictions
        for k in range(N):
            t_predict = t + step_horizon * k
            if type == "circle":
                angle = 4*math.pi/10*t_predict
                x_ref_ = 1.5*math.cos(angle)
                y_ref_ = 1.5*math.sin(angle)
                theta_ref_ = 0.0

            if type == "8shaped":
                x_ref_ = 1.5 + 1.5* np.sin(2*np.pi*t_predict/25)
                y_ref_ = 1.5*np.sin(4*np.pi*t_predict/25)
                theta_ref_ = 0.0
            if type=="traj1":
                if t_predict <= 6:
                    x_ref_ = 0.5*t_predict
                    y_ref_ = 0.0
                    theta_ref_ = 0.0
                if (t_predict > 6) and (t_predict <= 8):
                    x_ref_ = 3.0
                    y_ref_ = 0.0
                    theta_ref_ = 0.785*(t_predict-6)
                if (t_predict > 6):
                    x_ref_ = 3.0
                    y_ref_ = 0.0
                    theta_ref_ = 1.57
                # if (t_predict > 8) and (t_predict <= 14):
                #     x_ref_ = 3.0+0.5*(t_predict-8)
                #     y_ref_ = 0.0
                #     theta_ref_ = 3.14
                # if (t_predict > 14) and (t_predict <= 16):
                #     x_ref_ = 0.0
                #     y_ref_ = 0.0
                #     theta_ref_ = 3.14-1.57*(t_predict-14)
                # if (t_predict > 16):
                #     x_ref_ = 6.0
                #     y_ref_ = 0.0
                #     theta_ref_ = 0.0
            if type=="traj2":
                if t_predict <= 6:
                    x_ref_ = 0.5*t_predict
                    y_ref_ = 0.0
                    theta_ref_ = 0.0
                if (t_predict > 6) and (t_predict <= 12):
                    x_ref_ = 3.0
                    y_ref_ = 0.5*(t_predict-6)
                    theta_ref_ = 0.0
                if (t_predict > 12) and (t_predict <= 18):
                    x_ref_ = 3.0-0.5*(t_predict-12)
                    y_ref_ = 3.0
                    theta_ref_ = 0.0
                if (t_predict > 18) and (t_predict <= 24):
                    x_ref_ = 0.0
                    y_ref_ = 3-0.5*(t_predict-18)
                    theta_ref_ = 0.0
            if type=="traj3":
                x_ref_ = 0.0
                y_ref_ = 0.0
                theta_ref_ = np.pi/4*t_predict
            if type=="traj4":
                if t_predict <= 6:
                    x_ref_ = 0.5*t_predict
                    y_ref_ = 0.0
                    theta_ref_ = 0.0
                if (t_predict > 6) and (t_predict <= 12):
                    x_ref_ = 3.0-0.5*(t_predict-6)
                    y_ref_ = 0.0
                    theta_ref_ = 0.0
                if (t_predict > 12):
                    x_ref_ = 6.0
                    y_ref_ = 0.0
                    theta_ref_ = 0.0
            x.append(x_ref_)
            x.append(y_ref_)
            x.append(theta_ref_)
            x_ref.append(x_ref_)
            y_ref.append(y_ref_)
            theta_ref.append(theta_ref_)
            vx, vy, vth = self.rabbit_model.velocity_from_discrete_points(k, 0.1, x_ref, y_ref, theta_ref)
            v_ref1, v_ref2, v_ref3, v_ref4 = self.rabbit_model.inverse_kinematic(vx, vy, vth, "numpy")
            u.append(v_ref1)
            u.append(v_ref2)
            u.append(v_ref3)
            u.append(v_ref4)

        # reshaped state and control
        x = np.array(x).reshape(N+1, -1)
        u = np.array(u).reshape(N, -1)

        return x, u

    def casadi_model_setup(self, type="track"):
        if type=="point":
            x = ca.SX.sym('x')
            y = ca.SX.sym('y')
            theta = ca.SX.sym('theta')
            states = ca.vertcat(x, y, theta)
            n_states = states.numel()
            ## Controls
            v1 = ca.SX.sym('v1')
            v2 = ca.SX.sym('v2')
            v3 = ca.SX.sym('v3')
            v4 = ca.SX.sym('v4')
            controls = ca.vertcat(v1, v2, v3, v4)
            n_controls = controls.numel()

            # Matrix containing all states
            X = ca.SX.sym('X', n_states, self.N+1)
            # Matrix containing all controls
            U = ca.SX.sym('U', n_controls, self.N)
            # Matrix containing
            P = ca.SX.sym('P', n_states+n_states)
            # State weight matrix
            Q = np.diag(self.mat_Q)
            # Control weight matrix
            R = np.diag(self.mat_R)
            # RHS

            RHS = self.rabbit_model.forward_kinematic(v1, v2, v3, v4, theta, "sym")
            # nonlinear function for mpc model
            f = ca.Function('f', [states, controls], [RHS])
            # cost and constraint
            cost_fn = 0
            g = X[:, 0] - X_ref[:, 0]
            # Euler
            for k in range(self.N):
                st = X[:, k] - P[:, k+1]
                con = U[:, k]
                cost_fn = cost_fn + st.T@Q@st + con.T@R@con
                st_next = X[:, k+1]
                f_value = f(X[:, k], U[:, k])
                st_next_euler = X[:, k] + (self.dt*f_value)
                g = ca.vertcat(g, st_next-st_next_euler)

            optimal_var = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
            optimal_par = ca.vertcat(ca.reshape(X_ref, -1, 1), ca.reshape(U_ref, -1, 1))
            nlp_prob = {'f': cost_fn, 'x': optimal_var, 'p': optimal_par, 'g': g}
            opts = {
                    'ipopt.max_iter': 5000,
                    'ipopt.print_level': 0,
                    'ipopt.acceptable_tol': 1e-6,
                    'ipopt.acceptable_obj_change_tol': 1e-4,
                    'print_time': 0}
            solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

            lbx = ca.DM.zeros((n_states*(self.N+1) + n_controls*self.N, 1))
            ubx = ca.DM.zeros((n_states*(self.N+1) + n_controls*self.N, 1))

            lbx[0: n_states*(self.N+1): n_states] = self.lowX[0]
            lbx[1: n_states*(self.N+1): n_states] = self.lowX[1]
            lbx[2: n_states*(self.N+1): n_states] = self.lowX[2]


            ubx[0: n_states*(self.N+1): n_states] = self.highX[0]
            ubx[1: n_states*(self.N+1): n_states] = self.highX[1]
            ubx[2: n_states*(self.N+1): n_states] = self.highX[2]


            lbx[n_states*(self.N+1): n_states*(self.N+1)+n_controls*self.N: 4] = self.lowU[0]
            lbx[n_states*(self.N+1)+1: n_states*(self.N+1)+n_controls*self.N: 4] = self.lowU[1]
            lbx[n_states*(self.N+1)+2: n_states*(self.N+1)+n_controls*self.N: 4] = self.lowU[2]
            lbx[n_states*(self.N+1)+3: n_states*(self.N+1)+n_controls*self.N: 4] = self.lowU[3]


            ubx[n_states*(self.N+1): n_states*(self.N+1)+n_controls*self.N: 4] = self.highU[0]
            ubx[n_states*(self.N+1)+1: n_states*(self.N+1)+n_controls*self.N: 4] = self.highU[1]
            ubx[n_states*(self.N+1)+2: n_states*(self.N+1)+n_controls*self.N: 4] = self.highU[2]
            ubx[n_states*(self.N+1)+3: n_states*(self.N+1)+n_controls*self.N: 4] = self.highU[3]


            args = {
                'lbg': ca.DM.zeros((n_states*(self.N+1), 1)),
                'ubg': ca.DM.zeros((n_states*(self.N+1), 1)),
                'lbx': lbx,
                'ubx': ubx
            }

            return f, solver, args

        if type=="track":
            ## States
            x = ca.SX.sym('x')
            y = ca.SX.sym('y')
            theta = ca.SX.sym('theta')
            states = ca.vertcat(x, y, theta)
            n_states = states.numel()
            ## Controls
            v1 = ca.SX.sym('v1')
            v2 = ca.SX.sym('v2')
            v3 = ca.SX.sym('v3')
            v4 = ca.SX.sym('v4')
            controls = ca.vertcat(v1, v2, v3, v4)
            n_controls = controls.numel()

            # Matrix containing all states
            X = ca.SX.sym('X', n_states, self.N+1)
            # Matrix containing all controls
            U = ca.SX.sym('U', n_controls, self.N)
            # Matrix containing all states_ref
            X_ref = ca.SX.sym('X_ref', n_states, self.N+1)
            # Matrix containing all controls_ref
            U_ref = ca.SX.sym('U_ref', n_controls, self.N)
            # State weight matrix
            Q = np.diag(self.mat_Q)
            # Control weight matrix
            R = np.diag(self.mat_R)

            RHS = self.rabbit_model.forward_kinematic(v1, v2, v3, v4, theta, "sym")

            # nonlinear function for mpc model
            f = ca.Function('f', [states, controls], [RHS])
            # cost and constraint
            cost_fn = 0
            g = X[:, 0] - X_ref[:, 0]
            # Euler
            for k in range(self.N):
                st_err = X[:, k] - X_ref[:, k]
                con_err = U[:, k] - U_ref[:, k]
                cost_fn = cost_fn + st_err.T @ Q @ st_err + con_err.T @ R @ con_err
                st_next = X[:, k+1]
                st_next_euler = X[:, k] + self.dt*f(X[:, k], U[:, k])
                g = ca.vertcat(g, st_next-st_next_euler)

            cost_fn = cost_fn + (X[:, self.N]-X_ref[:, self.N]).T@Q@(X[:, self.N]-X_ref[:, self.N])

            optimal_var = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
            optimal_par = ca.vertcat(ca.reshape(X_ref, -1, 1), ca.reshape(U_ref, -1, 1))
            nlp_prob = {'f': cost_fn, 'x': optimal_var, 'p': optimal_par, 'g': g}
            opts = {
                    'ipopt.max_iter': 5000,
                    'ipopt.print_level': 0,
                    'ipopt.acceptable_tol': 1e-6,
                    'ipopt.acceptable_obj_change_tol': 1e-4,
                    'print_time': 0}
            solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

            lbx = ca.DM.zeros((n_states*(self.N+1) + n_controls*self.N, 1))
            ubx = ca.DM.zeros((n_states*(self.N+1) + n_controls*self.N, 1))

            lbx[0: n_states*(self.N+1): n_states] = self.lowX[0]
            lbx[1: n_states*(self.N+1): n_states] = self.lowX[1]
            lbx[2: n_states*(self.N+1): n_states] = self.lowX[2]


            ubx[0: n_states*(self.N+1): n_states] = self.highX[0]
            ubx[1: n_states*(self.N+1): n_states] = self.highX[1]
            ubx[2: n_states*(self.N+1): n_states] = self.highX[2]


            lbx[n_states*(self.N+1)  : n_states*(self.N+1)+n_controls*self.N: 4] = self.lowU[0]
            lbx[n_states*(self.N+1)+1: n_states*(self.N+1)+n_controls*self.N: 4] = self.lowU[1]
            lbx[n_states*(self.N+1)+2: n_states*(self.N+1)+n_controls*self.N: 4] = self.lowU[2]
            lbx[n_states*(self.N+1)+3: n_states*(self.N+1)+n_controls*self.N: 4] = self.lowU[3]


            ubx[n_states*(self.N+1)  : n_states*(self.N+1)+n_controls*self.N: 4] = self.highU[0]
            ubx[n_states*(self.N+1)+1: n_states*(self.N+1)+n_controls*self.N: 4] = self.highU[1]
            ubx[n_states*(self.N+1)+2: n_states*(self.N+1)+n_controls*self.N: 4] = self.highU[2]
            ubx[n_states*(self.N+1)+3: n_states*(self.N+1)+n_controls*self.N: 4] = self.highU[3]


            args = {
                'lbg': ca.DM.zeros((n_states*(self.N+1), 1)),
                'ubg': ca.DM.zeros((n_states*(self.N+1), 1)),
                'lbx': lbx,
                'ubx': ubx
            }

            return f, solver, args


    def casadi_nmpc_solver(self, initX, initU, type=None, run_type=None):
        # simulation
        t0 = 0
        mpciter = 0
        init_state = np.array([initX[0], initX[1], initX[2]]).reshape(1,-1)
        current_state = init_state.copy()
        init_control = np.array([initU[0], initU[1], initU[2], initU[3]]).reshape(1, -1)
        state = np.tile(current_state.reshape(-1, 1), self.N+1).T
        control = np.tile(init_control.reshape(-1, 1), self.N).T
        next_trajectories = state.copy()
        next_controls = control.copy()
        f, solver, args = self.casadi_model_setup()
        lbx = args['lbx']
        ubx = args['ubx']
        if run_type=="test":
            while (mpciter * self.dt < self.sim_time):
                current_time = mpciter * self.dt
                args['p'] = np.concatenate((
                    next_trajectories.reshape(-1, 1),
                    next_controls.reshape(-1, 1))
                )
                args['x0'] = np.concatenate(
                    (state.reshape(-1,1),
                    control.reshape(-1,1))
                )
                sol = solver(
                    x0=args['x0'],
                    p = args['p'],
                    lbx=args['lbx'],
                    ubx=args['ubx'],
                    lbg=args['lbg'],
                    ubg=args['ubg'],
                )
                sol_x = ca.reshape(sol['x'][:3*(self.N+1)], 3, self.N+1)
                sol_u = ca.reshape(sol['x'][3*(self.N+1):], 4, self.N)
                t0, current_state, state, control = self.shift_timestep(self.dt, t0, current_state, sol_x, sol_u, f)
                next_trajectories, next_controls = self.reference_state_and_control(t0, self.dt, current_state, self.N, type=type)
                print(f" optimal state :x = {np.round(sol_x.full()[0, self.index], 3)}, y = {np.round(sol_x.full()[1, self.index], 3)}, theta = {np.round(sol_x.full()[2, self.index], 3)}")
                v1_m1, v2_m2, v3_m3, v4_m4 = sol_u.full()[0, self.index], sol_u.full()[1, self.index], sol_u.full()[2, self.index], sol_u.full()[3, self.index]
                #print(v1_m1, v2_m2, v3_m3, v4_m4)
                mpciter += 1

        elif run_type=="ros2":
            current_time = mpciter * self.dt
            args['p'] = np.concatenate((
                next_trajectories.reshape(-1, 1),
                next_controls.reshape(-1, 1))
            )
            args['x0'] = np.concatenate(
                (state.reshape(-1,1),
                control.reshape(-1,1))
            )
            sol = solver(
                x0=args['x0'],
                p = args['p'],
                lbx=args['lbx'],
                ubx=args['ubx'],
                lbg=args['lbg'],
                ubg=args['ubg'],
            )
            sol_x = ca.reshape(sol['x'][:3*(self.N+1)], 3, self.N+1)
            sol_u = ca.reshape(sol['x'][3*(self.N+1):], 4, self.N)
            t0, current_state, state, control = self.shift_timestep(self.dt, t0, current_state, sol_x, sol_u, f)
            next_trajectories, next_controls = self.reference_state_and_control(t0, self.dt, current_state, self.N, type=type)
            # print(f" optimal state :x = {np.round(sol_x.full()[0, self.index], 3)}, y = {np.round(sol_x.full()[1, self.index], 3)}, theta = {np.round(sol_x.full()[2, self.index], 3)}")
            # v1_m1, v2_m2, v3_m3, v4_m4 = sol_u.full()[0, self.index], sol_u.full()[1, self.index], sol_u.full()[2, self.index], sol_u.full()[3, self.index]
            print(v1_m1, v2_m2, v3_m3, v4_m4 )
            # print()
            mpciter += 1


if __name__=="__main__":
    initX = [0.0, 0.0, 0.0]
    initU = [0.0, 0.0, 0.0, 0.0]
    lowX =  [-5.0, -5.0,  -np.pi]
    highX = [5.0, 5.0,  np.pi]
    lowU = [-30, -30, -30, -30]
    highU =[ 30,  30,  30,  30]
    mat_Q = [5000, 5000, 5000]
    mat_R = [0.001, 0.001, 0.001, 0.001]

    controller = CasadiNMPC(
        initX=initX, initU=initU,
        lowX=lowX, highX=highX,
        lowU=lowU, highU=highU,
        mat_Q=mat_Q, mat_R=mat_R,
        N=50, dt=0.1, sim_time = 15
    )
    controller.casadi_nmpc_solver(
        initX, initU, type="traj1", run_type="test"
    )
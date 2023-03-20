import rclpy
import numpy as np
import casadi as ca
import time

from .library.casadi_solver_rabbit import CasadiNMPC
from .library.rabbit_robot import RabbitModel
from .library.bezier_path import calc_4points_bezier_path, calc_bezier_path

from rclpy.action import ActionServer
from rclpy.node import Node
from mpc_action.action import MPCAction
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist, Pose, Vector3

class MPCActionServer(Node):

    def __init__(self):
        super().__init__('mpc_server_node')
        self.mpc_server = ActionServer(
                self,
                MPCAction,
                'mpc_action',
                self.mpc_callback)
        self.initX = [0.0, 0.0, 0.0]
        self.initU = [0.0, 0.0, 0.0, 0.0]

        self.lowX =  [-3.0, -3.0, -np.pi]
        self.highX = [ 3.0,  3.0,  np.pi]

        self.lowU = [-10, -10, -10, -10]
        self.highU = [10, 10, 10, 10]

        self.mat_Q = [75, 75, 90]
        self.mat_R = [0.1, 0.1, 0.1, 0.1]

        self.mpc_type = "circle"
        self.index = 0
        self.N = 100
        self.dt = 0.1
        self.t0 = 0
        self.mpciter = 0
        self.sim_time = 23
        self.speed_up = lambda t: 10*(1-np.exp(-2*t))

        ## States
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vth = 0.0
        self.feedback_states = np.array([
                                        self.current_x,
                                        self.current_y,
                                        self.current_yaw])
        self.current_states = self.feedback_states.copy()

        ## Controls
        self.current_w1 = 0.0
        self.current_w2 = 0.0
        self.current_w3 = 0.0
        self.current_w4 = 0.0
        self.feedback_controls = np.array([
                                        self.current_w1,
                                        self.current_w2,
                                        self.current_w3,
                                        self.current_w4])

        self.states = np.tile(self.feedback_states.reshape(-1,1), self.N+1).T
        self.controls = np.tile(self.feedback_controls.reshape(-1,1), self.N).T
        self.next_trajectories = self.states.copy()
        self.next_controls = self.controls.copy()
        self.casadi_solver = CasadiNMPC(
                            initX=self.feedback_states, initU=self.feedback_controls[0],
                            lowX=self.lowX, highX=self.highX,
                            lowU=self.lowU, highU=self.highU,
                            mat_Q=self.mat_Q, mat_R=self.mat_R,
                            N=self.N, dt=self.dt, sim_time=self.sim_time)
        
        ## Path Planner Goal
        #self.goal_x = 0.0
        #self.goal_y = 0.0
        #self.goal_yaw = 0.0
        self.n_points = 100
        self.offset = 2.0
        #self.start_X = [self.feedback_states[0], self.feedback_states[1], self.feedback_states[2]]
        #self.end_X = [self.goal_x, self.goal_y, self.goal_yaw]

        #self.path, _ = self.calc_planner(self.start_X, self.end_X, self.n_points, self.offset)

        #self.path_x = self.path[:, 0]
        #self.path_y = self.path[:, 1]
        #self.path_yaw = np.append(np.arctan2(np.diff(self.path_y), np.diff((self.path_x))), self.goal_yaw)

        #self.goal_states = np.vstack([self.path_x, self.path_y, self.path_yaw]).T

        ## Solver
        self.f, self.solver, self.args = self.casadi_solver.casadi_model_setup()
        self.rabbit_model = RabbitModel()

        ## Euclidean norm condition
        self.norm_cond = 0.0

        ## ROS SETUP
        mpc_timer = 0.1
        control_timer = 0.01
        cmd_timer = 0.01
        self.odom_subscriber = self.create_subscription(Vector3, 'odometry', self.odom_callback, 100)
        self.control_subscirber = self.create_subscription(Float32MultiArray, 'control_feedback', self.controls_callback, 100)
        self.control_publisher = self.create_publisher(Float32MultiArray, 'input_control', 100)
        self.cmd_control_publisher = self.create_publisher(Twist, 'cmd_vel', 100)
        # self.cmd_timer = self.create_timer(cmd_timer, self.cmd_control_callback)
        #self.solver_time = self.create_timer(mpc_timer, self.nmpc_solver)


    def odom_callback(self, odom_msg):
        self.current_x = odom_msg.x
        self.current_y = odom_msg.y
        self.current_yaw = odom_msg.z

        self.feedback_states = np.array([
                                        self.current_x,
                                        self.current_y,
                                        self.current_yaw])

    def controls_callback(self, con_msg):
        self.current_w1 = con_msg.data[0]
        self.current_w2 = con_msg.data[1]
        self.current_w3 = con_msg.data[2]
        self.current_w4 = con_msg.data[3]
        self.feedback_controls = np.array([
                                            self.current_w1,
                                            self.current_w2,
                                            self.current_w3,
                                            self.current_w4])

    def calc_planner(self, start_X, end_X, n_points, offset):
        dist = np.hypot(start_X[0] - end_X[0], start_X[1]-end_X[1]) / offset
        control_points =  np.array([
            [start_X[0], start_X[1]],
            [start_X[0] + dist * np.cos(start_X[2]), start_X[1] + dist * np.sin(start_X[2])],
            [end_X[0] - dist * np.cos(end_X[2]), end_X[1] - dist * np.sin(end_X[2])],
            [end_X[0], end_X[1]]
            ])

        path = calc_bezier_path(control_points, n_points)

        return path, control_points
    
    def path_beizer(self, goal_states):


        self.start_X = [self.feedback_states[0], self.feedback_states[1], self.feedback_states[2]]
        self.end_X = [goal_states[0], goal_states[1], goal_states[2]]

        path, _ = self.calc_planner(self.start_X, self.end_X, self.n_points, self.offset)
        path_x = path[:, 0]
        path_y = path[:, 1]
        path_yaw = np.append(np.arctan2(np.diff(path_y), np.diff((path_x))), goal_states[2])

        return np.vstack([path_x, path_y, path_yaw]).T
    
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

    def mpc_callback(self, goal_handle):
        self.get_logger().info('Executing task...')

        goal_feedback = MPCAction.Feedback()
        control_result = MPCAction.Result()

        goal = np.array(goal_handle.request.goal_states)
        # print(goal)
        goal_states = self.path_beizer(goal)



        while (np.linalg.norm(goal_states[goal_states.shape[0]-1] - self.current_states, 2) > 0.01):
            if np.linalg.norm(goal_states[goal_states.shape[0]-1] - self.current_states, 2) > 0.3:
                self.norm_cond += 0.06
                if self.norm_cond % 3 == 0:
                    self.norm_cond = 3.0
                self.args['lbx'][3*(self.N+1):] = -self.speed_up(self.norm_cond)
                self.args['ubx'][3*(self.N+1):] =  self.speed_up(self.norm_cond)
            else:
                self.args['lbx'][3*(self.N+1):] = self.lowU[0]
                self.args['ubx'][3*(self.N+1):] = self.highU[0]
                self.norm_cond = 0.0
            self.args['p'] = np.concatenate([
                self.next_trajectories.reshape(-1,1),
                self.next_controls.reshape(-1,1)])
            self.args['x0'] = np.concatenate([
                self.states.reshape(-1,1),
                self.controls.reshape(-1,1)])

            sol = self.solver(
                    x0=self.args['x0'],
                    p =self.args['p'],
                    lbx=self.args['lbx'],
                    ubx=self.args['ubx'],
                    lbg=self.args['lbg'],
                    ubg=self.args['ubg'])

            sol_x = ca.reshape(sol['x'][:3*(self.N+1)], 3, self.N+1)
            sol_u = ca.reshape(sol['x'][3*(self.N+1):], 4, self.N)
            # print(sol_x.full()[:, self.index])
            ########################### Shift Timestep ###########################
            # self.t0 = self.t0 + self.dt
            # f_value = self.f(self.feedback_states, self.feedback_controls)
            # self.feedback_states = self.feedback_states + self.dt * f_value
            # self.states = np.tile(self.feedback_states.full().reshape(-1, 1), self.N+1).T
            # self.controls = np.tile(self.feedback_controls.reshape(-1, 1), self.N).T
            self.t0, self.feedback_states, self.states, self.controls = self.shift_timestep(self.dt, self.t0, self.feedback_states, sol_x, sol_u, self.f)
            ################################################## Apply Next Prediction ##################################################
            for k in range(self.N):
                index = self.mpciter + k
                if index >= goal_states.shape[0]:
                    index = goal_states.shape[0]-1
                self.next_trajectories[0] = self.current_states.T
                self.next_trajectories[k+1] = goal_states[index, :]
                self.next_controls = np.tile(np.array([10, 10, 10, 10]).reshape(1, -1), self.N).T
            ###########################################################################################################################
            v1_m1, v2_m2, v3_m3, v4_m4 = sol_u.full()[0, self.index], sol_u.full()[1, self.index], sol_u.full()[2, self.index], sol_u.full()[3, self.index]
            self.current_x = sol_x.full()[0, self.index]
            self.current_y = sol_x.full()[1, self.index]
            self.current_yaw = sol_x.full()[2, self.index]
            self.feedback_states = np.array([
                                        self.current_x,
                                        self.current_y,
                                        self.current_yaw])
            self.current_w1 = v1_m1
            self.current_w2 = v2_m2
            self.current_w3 = v3_m3
            self.current_w4 = v4_m4
            self.feedback_controls = np.array([
                                            self.current_w1,
                                            self.current_w2,
                                            self.current_w3,
                                            self.current_w4])
            goal_feedback.goal_feedback = [self.current_x, self.current_y, self.current_yaw]
            goal_handle.publish_feedback(goal_feedback)
            self.current_vx, self.current_vy, self.current_vth = self.rabbit_model.forward_kinematic(
                                                                                                    v1_m1,
                                                                                                    v2_m2,
                                                                                                    v3_m3,
                                                                                                    v4_m4,
                                                                                                    sol_x.full()[2, self.index],
                                                                                                    "numpy")
            print(self.feedback_states)
            # print(self.feedback_controls)
            self.mpciter += 1
            time.sleep(0.1)



        if (np.linalg.norm(goal_states[goal_states.shape[0]-1] - self.current_states, 2) == 0.01):
            goal_handle.succeed()

        result = MPCAction.Result()
        result.goal_results = goal_feedback.goal_feedback


        return result


        


def main(args=None):
    rclpy.init(args=args)

    mpc_server = MPCActionServer()

    rclpy.spin(mpc_server)


if __name__=='__main__':
    main()
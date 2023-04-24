import rclpy
import numpy as np
import casadi as ca

from .library.casadi_solver_rabbit import CasadiNMPC
from .library.rabbit_robot import RabbitModel
from .library.bezier_path import calc_4points_bezier_path, calc_bezier_path
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist, Pose, Vector3


class NMPCRabbit(Node):

    def __init__(self):
        super().__init__('nmpc_rabbit')
        ## NMPC Params
        self.initX = [0.0, 0.0, 0.0]
        self.initU = [0.0, 0.0, 0.0, 0.0]

        self.lowX =  [-3.0, -3.0, -np.pi]
        self.highX = [ 3.0,  3.0,  np.pi]

        self.lowU = [-10, -10, -10, -10]
        self.highU = [10, 10, 10, 10]

        self.mat_Q = [75, 75, 90]
        self.mat_R = [0.01, 0.01, 0.01, 0.01]

        self.mpc_type = "circle"
        self.index = 0
        self.N = 50
        self.dt = 0.1
        self.t0 = 0
        self.mpciter = 0
        self.sim_time = 23
        self.speed_up = lambda t: 10*(1-np.exp(-2*t))

        ## Simulation
        ### States
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        # self.current_vx = 0.0
        # self.current_vy = 0.0
        # self.current_vth = 0.0
        self.feedback_states = np.array([
                                        self.current_x,
                                        self.current_y,
                                        self.current_yaw
                                    ])
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
                                        self.current_w4
                                    ])
        self.init_states = self.feedback_states.copy()
        self.states = np.tile(self.feedback_states.reshape(-1, 1), self.N+1).T
        self.controls = np.tile(self.feedback_controls.reshape(-1, 1), self.N).T
        self.next_trajectories = self.states.copy()
        self.next_controls = self.controls.copy()
        self.casadi_solver = CasadiNMPC(
            initX=self.feedback_states, initU=self.initU,
            lowX=self.lowX, highX=self.highX,
            lowU=self.lowU, highU=self.highU,
            mat_Q=self.mat_Q, mat_R=self.mat_R,
            N=self.N, dt=self.dt, sim_time = self.sim_time
        )
        ## Path Planner goal
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0
        self.n_points = 100
        self.offset = 2.0
        self.start_X = [self.feedback_states[0], self.feedback_states[1], self.feedback_states[2]]
        self.end_X = [self.goal_x, self.goal_y, self.goal_yaw]

        self.path, _ = self.calc_planner(self.start_X, self.end_X, self.n_points, self.offset)

        self.path_x = self.path[:, 0]
        self.path_y = self.path[:, 1]
        self.path_yaw = np.append(np.arctan2(np.diff(self.path_y), np.diff((self.path_x))), self.goal_yaw)

        self.goal_states = np.vstack([self.path_x, self.path_y, self.path_yaw]).T

        ## Solver
        self.f, self.solver, self.args = self.casadi_solver.casadi_model_setup()
        self.rabbit_model = RabbitModel()
        ## Euclidean norm condition
        self.norm_cond = 0.0

        # Optimal Control
        self.opt_u1 = 0.0
        self.opt_u2 = 0.0
        self.opt_u3 = 0.0
        self.opt_u4 = 0.0

        ## ROS Setup
        mpc_timer = 0.1
        control_timer = 0.01
        self.odom_subscriber = self.create_subscription(Vector3, 'odometry', self.odom_callback, 10)
        self.control_subscriber = self.create_subscription(Float32MultiArray, 'feedback_controls', self.controls_callback, 10)
        self.control_publisher = self.create_publisher(Float32MultiArray, 'input_controls', 10)
        self.path_gen = self.create_subscription(Float32MultiArray, 'path_gen', self.path_callback, 10)
        self.control_timer = self.create_timer(control_timer, self.control_timer_pub)
        self.solver_time = self.create_timer(mpc_timer, self.nmpc_solver)


    def odom_callback(self, odom_msg):
        self.current_x = odom_msg.x
        self.current_y = odom_msg.y
        self.current_yaw = odom_msg.z

        self.feedback_states = np.array([
                                        self.current_x,
                                        self.current_y,
                                        self.current_yaw
                                    ])
        
        # self.get_logger().info("Current states '%s'" % odom_msg)

    def controls_callback(self, con_msg):
        self.current_w1 = con_msg.data[0]
        self.current_w2 = con_msg.data[1]
        self.current_w3 = con_msg.data[2]
        self.current_w4 = con_msg.data[3]
        self.feedback_controls = np.array([
                                        self.current_w1,
                                        self.current_w2,
                                        self.current_w3,
                                        self.current_w4
                                    ])

    def path_callback(self, path_msg):

        self.goal_x = path_msg.data[0]
        self.goal_y = path_msg.data[1]
        self.goal_yaw = path_msg.data[2]

        self.start_X = [self.feedback_states[0], self.feedback_states[1], self.feedback_states[2]]
        self.end_X = [self.goal_x, self.goal_y, self.goal_yaw]

        self.path, _ = self.calc_planner(self.start_X, self.end_X, self.n_points, self.offset)

        self.path_x = self.path[:, 0]
        self.path_y = self.path[:, 1]
        self.path_yaw = np.append(np.arctan2(np.diff(self.path_y), np.diff((self.path_x))), self.goal_yaw)

        self.goal_states = np.vstack([self.path_x, self.path_y, self.path_yaw]).T

        # print(self.goal_states)

        # self.get_logger().info("Path is being calculated '%s'" % path_msg)


    def nmpc_solver(self):
        start_time = self.get_clock().now()
        if np.linalg.norm(self.goal_states[self.goal_states.shape[0]-1, :]-self.feedback_states, 2) > 0.3:
            self.norm_cond += 0.06
            if self.norm_cond % 3 == 0:
                self.norm_cond = 3.0
            self.args['lbx'][3*(self.N+1):] = -self.speed_up(self.norm_cond)
            self.args['ubx'][3*(self.N+1):] = self.speed_up(self.norm_cond)
        else:
            self.args['lbx'][3*(self.N+1):] = self.lowU[0]
            self.args['ubx'][3*(self.N+1):] = self.highU[0]
            self.norm_cond = 0.0
        self.args['p'] = np.concatenate([
            self.next_trajectories.reshape(-1, 1),
            self.next_controls.reshape(-1, 1)
        ])
        self.args['x0'] = np.concatenate([
            self.states.reshape(-1, 1),
            self.controls.reshape(-1, 1)
        ])
        sol = self.solver(
            x0= self.args['x0'],
            p = self.args['p'],
            lbx=self.args['lbx'],
            ubx=self.args['ubx'],
            lbg=self.args['lbg'],
            ubg=self.args['ubg'],
        )
        sol_x = ca.reshape(sol['x'][:3*(self.N+1)], 3, self.N+1)
        sol_u = ca.reshape(sol['x'][3*(self.N+1):], 4, self.N)


        # print(sol_x.full()[:, self.index])
        # print(sol_u.full()[:, self.index])
        ################################################## Obtained Optimal Control ##################################################
        self.opt_u1 = sol_u.full()[0, self.index]
        self.opt_u2 = sol_u.full()[1, self.index]
        self.opt_u3 = sol_u.full()[2, self.index]
        self.opt_u4 = sol_u.full()[3, self.index]
        ##############################################################################################################################
        ################################################## Shift Timestep ##################################################
        self.t0 = self.t0 + self.dt
        f_value = self.f(self.feedback_states, self.feedback_controls)
        self.current_states = self.feedback_states + self.dt * f_value
        self.states = np.tile(self.current_states.full().reshape(-1, 1), self.N+1).T
        self.controls = np.tile(self.feedback_controls.reshape(-1, 1), self.N).T
        ################################################## Apply Next Prediction ##################################################
        for k in range(self.N):
            index = self.mpciter + k
            if index >= self.goal_states.shape[0]:
                index = self.goal_states.shape[0]-1
            self.next_trajectories[0] = self.feedback_states.T
            self.next_trajectories[k+1] = self.goal_states[index, :]
            self.next_controls = np.tile(np.array([10, 10, 10, 10]).reshape(1, -1), self.N).T
        ############################################################################################################################
        end_time = self.get_clock().now()

        duration = (end_time - start_time )

        print(duration)
        self.mpciter += 1
        

    def control_timer_pub(self):
        con_msg = Float32MultiArray()
        con_msg.data = [self.opt_u1, self.opt_u2, self.opt_u3, self.opt_u4]
        self.control_publisher.publish(con_msg)

        # self.get_logger().info("Publishing optimal control '%s'" % con_msg)


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


def main(args=None):
    rclpy.init(args=args)

    rabbit_go = NMPCRabbit()

    rclpy.spin(rabbit_go)
    rabbit_go.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

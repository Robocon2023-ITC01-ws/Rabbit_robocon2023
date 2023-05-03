import rclpy
import numpy as np
import casadi as ca

from .library.casadi_solver_rabbit import CasadiNMPC
from .library.rabbit_robot import RabbitModel
from .library.bezier_path import calc_4points_bezier_path, calc_bezier_path
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int64, Bool
from geometry_msgs.msg import Twist, Pose, Vector3
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion


class NMPCRabbit(Node):

    def __init__(self):
        super().__init__('nmpc_rabbit')
        ## NMPC Params
        self.initX = [0.0, 0.0, 0.0]
        self.initU = [0.0, 0.0, 0.0, 0.0]

        self.lowX =  [-6, -6, -3.14]
        self.highX = [ 6,  6, 3.14]

        self.lowU = [-30, -30, -30, -30]
        self.highU = [30, 30, 30, 30]

        self.mat_Q = [750, 750, 2000]
        self.mat_R = [1, 1, 1, 1]

        self.goal_flag = False

        self.mpc_type = "circle"
        self.index = 0
        self.N = 100
        self.dt = 0.1
        self.t0 = 0
        self.mpciter = 0
        self.sim_time = 23
        self.speed_up = lambda t: 30*(1-np.exp(-2*t))
        self.n_points = 100

        # Euler angle


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
                                    ], dtype=np.float64)
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
                                    ], dtype=np.float64)
        self.init_states = self.feedback_states.copy()
        self.states = np.tile(self.feedback_states.reshape(3, 1), self.N+1)
        self.controls = np.tile(self.feedback_controls.reshape(4, 1), self.N)
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

        self.goal_states = np.vstack([self.goal_x, self.goal_y, self.goal_yaw])

        ## Solver
        self.f, self.solver, self.args = self.casadi_solver.casadi_model_setup()
        self.rabbit_model = RabbitModel()
        ## Euclidean norm condition
        self.norm_cond = 0.0
        self.tick = 0
        self.reset_tick = False

        # Optimal Control
        self.opt_u1 = 0.0
        self.opt_u2 = 0.0
        self.opt_u3 = 0.0
        self.opt_u4 = 0.0

        ## ROS Setup
        mpc_timer = 0.1
        control_timer = 0.01
        self.odom_subscriber = self.create_subscription(Float32MultiArray, 'state_est', self.odom_callback, 10)
        self.tick_publisher = self.create_publisher(Int64, 'tick', 10)
        self.reset_tick_subscriber = self.create_subscription(Bool, 'reset_tick', self.reset_tick_callback, 10)
        self.control_subscriber = self.create_subscription(Float32MultiArray, 'feedback_encoder', self.controls_callback, 10)
        self.quaternion_subscriber = self.create_subscription(Imu, 'imu/data2', self.quaternion_callback, 10)
        self.control_publisher = self.create_publisher(Float32MultiArray, 'input_controls', 10)
        self.path_gen = self.create_subscription(Float32MultiArray, 'beizer_path', self.path_callback, 10)
        self.control_timer = self.create_timer(control_timer, self.control_timer_pub)
        self.solver_time = self.create_timer(mpc_timer, self.nmpc_solver)


    def odom_callback(self, odom_msg):
        self.current_x = odom_msg.data[0]
        self.current_y = odom_msg.data[1]
        # self.current_yaw = odom_msg.z


        self.feedback_states = np.array([
                                        self.current_x,
                                        self.current_y,
                                        self.current_yaw
                                    ])
        
        # print(self.feedback_states)

    def quaternion_callback(self, quat_msg):
        q1 = quat_msg.orientation.x
        q2 = quat_msg.orientation.y
        q3 = quat_msg.orientation.z
        q4 = quat_msg.orientation.w

        orient_list = [q1, q2, q3, q4]

        roll, pitch, yaw = euler_from_quaternion(orient_list)

        self.current_yaw = yaw
        
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

        self.goal_states = np.array([self.goal_x, self.goal_y, self.goal_yaw])


        # print(self.goal_states)


        # self.get_logger().info("Path is being calculated '%s'" % path_msg)

    def reset_tick_callback(self, tick_msg):
        self.reset_tick = tick_msg.data


    def nmpc_solver(self):
        tick_msg = Int64()
        start_time = self.get_clock().now()
        if np.linalg.norm(self.goal_states-self.current_states, 2) > 0.3:
            self.norm_cond += 0.05
            if np.linalg.norm(self.goal_states-self.current_states, 2) < 0.01:
                self.norm_cond = 0.0
            self.args['lbx'][3*(self.N+1):] = -self.speed_up(self.norm_cond)
            self.args['ubx'][3*(self.N+1):] = self.speed_up(self.norm_cond)
            # print(self.norm_cond)
        else:
            self.args['lbx'][3*(self.N+1):] = self.lowU[0]
            self.args['ubx'][3*(self.N+1):] = self.highU[0]
            self.norm_cond = 0.0
        self.args['p'] = np.concatenate([
            self.next_trajectories.T.reshape(-1, 1),
            self.next_controls.T.reshape(-1, 1)
        ])
        self.args['x0'] = np.concatenate([
            self.states.T.reshape(-1, 1),
            self.controls.T.reshape(-1, 1)
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
        # self.t0 = self.t0 + self.dt
        self.current_states = self.feedback_states + self.dt * self.rabbit_model.forward_kinematic(
             self.feedback_controls[0], self.feedback_controls[1],
             self.feedback_controls[2], self.feedback_controls[3],
             self.current_states[2], "numpy"
        )
        #self.current_states = self.feedback_states + self.dt * self.rabbit_model.forward_kinematic(
        #    self.opt_u1, self.opt_u2,
        #    self.opt_u3, self.opt_u4,
        #    sol_x[2, self.index]  ,"numpy"
        #)

        self.states = np.tile(self.current_states.reshape(3, 1), self.N+1)
        self.controls = np.tile(self.feedback_controls.reshape(4, 1), self.N)
        ################################################## Apply Next Prediction ##################################################
        for k in range(self.N):
            # index = self.mpciter + k
            # if index >= self.goal_states.shape[1]:
            #     index = self.goal_states.shape[1]-1
            self.next_trajectories[0, 0] = self.current_states[0]
            self.next_trajectories[1, 0] = self.current_states[1]
            self.next_trajectories[2, 0] = self.current_states[2]
            self.next_trajectories[0, k+1] = self.goal_states[0]
            self.next_trajectories[1, k+1] = self.goal_states[1]
            self.next_trajectories[2, k+1] = self.goal_states[2]
            # self.next_trajectories[:, k+1] = np.array([self.goal_x, self.goal_y, self.goal_yaw])
            #self.next_trajectories[:, k+1] = np.array([

            if (np.linalg.norm(self.current_states-self.goal_states, 2) > 0.01):
                 self.next_controls = np.tile(np.array([30, 30, 30, 30], dtype=np.float64).reshape(4, 1), self.N)
            elif ((np.linalg.norm(self.current_states-self.goal_states, 2) < 0.01)):
                 self.next_controls = np.tile(np.array([0, 0, 0, 0], dtype=np.float64).reshape(4, 1), self.N)
                
        tick_msg.data = self.tick

        if self.tick >= self.n_points:
            self.tick = self.n_points - 1

        # if np.linalg.norm(self.current_states-self.goal_states, 2) < 0.001:
        #     self.tick = 0
        self.tick_publisher.publish(tick_msg)

        

        ############################################################################################################################
        end_time = self.get_clock().now()

        duration = (end_time - start_time )
        self.tick += 1

        # print(self.opt_u1, self.opt_u2, self.opt_u3, self.opt_u4)

        # print(self.tick)
        

    def control_timer_pub(self):
        con_msg = Float32MultiArray()
        self.opt_u1 = np.round(self.opt_u1, 4)
        self.opt_u2 = np.round(self.opt_u2, 4)
        self.opt_u3 = np.round(self.opt_u3, 4)
        self.opt_u4 = np.round(self.opt_u4, 4)
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

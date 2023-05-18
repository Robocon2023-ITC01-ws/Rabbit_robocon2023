import rclpy
import numpy as np
import casadi as ca
import time

from .library.nmpc_solver import NMPCSolver
from .library.rabbit_omni import RabbitModel
from .library.bezier_path import calc_4points_bezier_path, calc_bezier_path
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion


class NMPCRabbit(Node):

    def __init__(self):
        super().__init__('nmpc_rabbit')
        ## NMPC Params

        self.lowX =  [-3, -3, -3.14]
        self.highX = [ 3,  3,  3.14]

        # self.lowX = [-ca.inf, -ca.inf, -ca.inf]
        # self.highX = [ca.inf, ca.inf, ca.inf]

        self.lowU = -20
        self.highU = 20

        self.mat_Q = [75, 75, 100]
        self.mat_R = [0.1, 0.1, 0.1, 0.1]


        self.goal_flag = False

        self.N = 50
        self.dt = 0.05
        self.t0 = 0
        self.mpciter = 0
        self.speed_up = lambda t: 20*(1-np.exp(-2*t))
        self.index = 0

        # Euler angle


        ## Simulation
        ### States
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vth = 0.0
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

        self.states = np.tile(self.feedback_states.reshape(3, 1), self.N+1)
        self.controls = np.tile(self.feedback_controls.reshape(4, 1), self.N)
        self.next_trajectories = self.states.copy()
        self.next_controls = self.controls.copy()
        ## Path Planner goal
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0
        self.n_points = 100
        self.offset = 20.0
        self.start_X = [self.feedback_states[0], self.feedback_states[1], self.feedback_states[2]]
        self.end_X = [self.goal_x, self.goal_y, self.goal_yaw]

        self.path, _ = self.calc_planner(self.start_X, self.end_X, self.n_points, self.offset)

        self.path_x = self.path[:, 0]
        self.path_y = self.path[:, 1]
        self.path_yaw = np.append(np.arctan2(np.diff(self.path_y), np.diff((self.path_x))), self.goal_yaw)

        self.goal_states = np.vstack([self.path_x, self.path_y, self.path_yaw])

        ## Solver
        self.f, self.solver, self.args = NMPCSolver(
            self.lowX, self.highX, self.lowU, self.highU,
            self.mat_Q, self.mat_R, self.N, self.dt
        ).nmpc_setup()

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
        self.odom_subscriber = self.create_subscription(Float32MultiArray, 'state_est', self.odom_callback, 10)
        self.control_subscriber = self.create_subscription(Float32MultiArray, 'feedback_encoder', self.controls_callback, 10)
        self.quaternion_subscriber = self.create_subscription(Imu, 'imu/data2', self.quaternion_callback, 10)
        self.control_publisher = self.create_publisher(Float32MultiArray, 'input_controls', 10)
        # self.twist_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        # self.twist_timer = self.create_timer(0.05, self.twist_command)
        self.path_gen = self.create_subscription(Float32MultiArray, 'path_gen', self.path_callback, 10)
        # self.control_timer = self.create_timer(control_timer, self.control_timer_pub)
        self.solver_time = self.create_timer(0.1, self.nmpc_solver)


    def quaternion_callback(self, quat_msg):
        q1 = quat_msg.orientation.x
        q2 = quat_msg.orientation.y
        q3 = quat_msg.orientation.z
        q4 = quat_msg.orientation.w

        orient_list = [q1, q2, q3, q4]

        roll, pitch, yaw = euler_from_quaternion(orient_list)

        self.current_yaw = yaw

    def odom_callback(self, odom_msg):
        self.current_x = odom_msg.data[0]
        self.current_y = odom_msg.data[1]
        # self.current_yaw = odom_msg.z


        self.feedback_states = np.array([
                                        self.current_x,
                                        self.current_y,
                                        self.current_yaw
                                    ])
        
        # self.get_logger().info("Current states '%s'" % odom_msg)

    def controls_callback(self, con_msg):
        
        # for_vec_tran = self.rabbit_model.forward_kinematic_tran(con_msg.data[0], con_msg.data[1],
        #                             con_msg.data[2], con_msg.data[3], 0.0, "numpy")
        
        # inv_vec = self.rabbit_model.inverse_kinematic(for_vec_tran[0], for_vec_tran[1], for_vec_tran[2], 0.0, "numpy")

        # self.current_w1 = inv_vec[0]
        # self.current_w2 = inv_vec[1]
        # self.current_w3 = inv_vec[2]
        # self.current_w4 = inv_vec[3]

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

        #self.path, _ = calc_4points_bezier_path(
        #    self.start_X[0], self.start_X[1], self.start_X[2],
        #    self.end_X[0], self.end_X[1], self.end_X[2],
        #    self.offset
        #)

        self.path_x = self.path[:, 0]
        self.path_y = self.path[:, 1]
        self.path_yaw = np.append(np.arctan2(np.diff(self.path_y), np.diff((self.path_x))), self.goal_yaw)

        self.goal_states = np.vstack([self.path_x, self.path_y, self.path_yaw])

        self.goal_flag = True

        # self.get_logger().info("Path is being calculated '%s'" % path_msg)


    def nmpc_solver(self):
        start_time = time.time()
        # self.goal_states[:, self.goal_states.shape[1]-1]
        if np.linalg.norm(np.array([self.goal_x, self.goal_y])-self.feedback_states[0:2], 2) > 0.3:
            self.norm_cond += 0.03
            self.args['lbx'][3*(self.N+1):] = -self.speed_up(self.norm_cond)
            self.args['ubx'][3*(self.N+1):] = self.speed_up(self.norm_cond)
            if np.linalg.norm(np.array([self.goal_x, self.goal_y])-self.feedback_states[0:2], 2) < 0.01:
                self.norm_cond = 0.0
        else:
            self.args['lbx'][3*(self.N+1):] = self.lowU
            self.args['ubx'][3*(self.N+1):] = self.highU
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
        self.opt_u1 = sol_u.full()[0, 0]
        self.opt_u2 = sol_u.full()[1, 0]
        self.opt_u3 = sol_u.full()[2, 0]
        self.opt_u4 = sol_u.full()[3, 0]
        ################################################## Apply Next Prediction ##################################################
        for k in range(self.N):
            self.index = self.mpciter + k
            if self.index >= self.goal_states.shape[1]:
                self.index = self.goal_states.shape[1]-1
            self.next_trajectories[0, 0] = self.feedback_states[0]
            self.next_trajectories[1, 0] = self.feedback_states[1]
            self.next_trajectories[2, 0] = self.feedback_states[2]
            self.next_trajectories[:, k+1] = self.goal_states[:, self.index]

            #self.next_trajectories[:, k+1] = np.array([self.goal_x, self.goal_y, self.goal_yaw])
            #self.next_trajectories[:, k+1] = np.array([

            if ((np.linalg.norm(self.feedback_states-self.goal_states[:, -1], 2) > 0.5) & (self.goal_flag)):
                 self.next_controls = np.tile(np.array([20, 20, 20, 20], dtype=np.float64).reshape(4, 1), self.N)
            elif ((np.linalg.norm(self.feedback_states-self.goal_states[:, -1], 2) < 0.05)):
                 self.next_controls = np.tile(np.array([0, 0, 0, 0], dtype=np.float64).reshape(4, 1), self.N)
        ############################################################################################################################
        ##############################################################################################################################
        ################################################## Shift Timestep ##################################################
        x_next = self.feedback_states + self.dt * self.rabbit_model.forward_kinematic_tran(
             self.feedback_controls[0], self.feedback_controls[1],
             self.feedback_controls[2], self.feedback_controls[3],
             self.feedback_states[2], "numpy"
        )

        self.current_states = x_next

        self.states = np.tile(self.feedback_states.reshape(3, 1), self.N+1)
        self.controls = np.tile(self.feedback_controls.reshape(4, 1), self.N)
        # if np.linalg.norm(self.goal_states[:, self.goal_states.shape[1]-1]-self.current_states, 2) > 0.01:
        #    self.mpciter =+ 4

        # elif np.linalg.norm(self.goal_states[:, self.goal_states.shape[1]-1]-self.current_states, 2) < 0.01:
        #    self.index = 0 
        #    self.mpciter =0
        
        # print(self.rabbit_model.forward_kinematic_tran(self.opt_u1, self.opt_u2, self.opt_u3, self.opt_u4, self.feedback_states[2], "numpy"))
        # print(self.opt_u1, self.opt_u2, self.opt_u3, self.opt_u4)
        # print(self.index)
        # print(self.args['x0'])
        con_msg = Float32MultiArray()
        con_msg.data = [self.opt_u1, self.opt_u2, self.opt_u3, self.opt_u4]
        self.control_publisher.publish(con_msg)
        self.mpciter +=1
        end_time = time.time()

        print(self.norm_cond)
        

        duration = (end_time - start_time )


    def control_timer_pub(self):
        con_msg = Float32MultiArray()
        
        # for_vec = self.rabbit_model.forward_kinematic(self.opt_u1, self.opt_u2, self.opt_u3, self.opt_u4, 0.0, "numpy")

        # inv_vec_tran = self.rabbit_model.inverse_kinematic_tran(for_vec[0], for_vec[1], for_vec[2], 0.0, "numpy")

        # con_msg.data = [inv_vec_tran[0], inv_vec_tran[1], inv_vec_tran[2], inv_vec_tran[3]]

        con_msg.data = [self.opt_u1, self.opt_u2, self.opt_u3, self.opt_u4]
        self.control_publisher.publish(con_msg)

        # self.get_logger().info("Publishing optimal control")

        # self.get_logger().info("Publishing optimal control '%s'" % con_msg)

    def twist_command(self):
        twist = Twist()
        
        self.vx, self.vy, self.vth = self.rabbit_model.forward_kinematic(self.opt_u1, self.opt_u2, self.opt_u3, self.opt_u4, 0.0, "numpy")

        #twist.linear.x = self.vx
        #twist.linear.y = self.vy
        #twist.angular.z = self.vth

        #self.twist_cmd.publish(twist)


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

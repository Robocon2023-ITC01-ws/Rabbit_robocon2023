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

        self.lowX =  [-6, -6, -3.14]
        self.highX = [ 6,  6,  3.14]

        self.lowU = -20
        self.highU = 20

        self.mat_Q = [1000, 1000, 2000]
        self.mat_R = [1, 1, 1, 1]


        self.N = 50
        self.dt = 0.1
        self.t0 = 0
        self.mpciter = 0
        self.speed_up = lambda t: 20*(1-np.exp(-2*t))
        self.n_points = 100

        ## ROS Setup
        mpc_timer = 0.1
        control_timer = 0.01
        self.wheel_subscriber = self.create_subscription(Float32MultiArray, 'odom_wheel', self.odom_callback, 10)
        # self.rotary_subscriber = self.create_subscription(Float32MultiArray, 'odometry_rotary', self.odom_callback, 10)
        # self.control_subscriber = self.create_subscription(Float32MultiArray, 'feedback_controls', self.controls_callback, 10)
        self.quaternion_subscriber = self.create_subscription(Imu, 'imu/data2', self.quaternion_callback, 10)
        self.control_publisher = self.create_publisher(Float32MultiArray, 'input_controls', 10)
        self.path_gen = self.create_subscription(Float32MultiArray, 'path_gen', self.path_callback, 10)
        self.control_timer = self.create_timer(control_timer, self.control_timer_pub)
        self.solver_time = self.create_timer(0.1, self.nmpc_solver)


        # Euler angle


        ## Simulation
        ### States
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
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
        self.offset = 3.0
        self.N_IND_SEARCH = 10
        self.prev_index = 0
        self.next_index = 0
        self.pred_index = self.prev_index + self.next_index
        self.index = 0
        self.dind = 0
        self.start_X = [self.feedback_states[0], self.feedback_states[1], self.feedback_states[2]]
        self.end_X = [self.feedback_states[0], self.feedback_states[1], self.feedback_states[2]]

        self.path, _ = self.calc_planner(self.start_X, self.end_X, self.n_points, self.offset)

        self.path_x = self.path[:, 0]
        self.path_y = self.path[:, 1]
        self.path_yaw = np.append(np.arctan2(np.diff(self.path_y), np.diff((self.path_x))), self.goal_yaw)

        self.goal_states = np.vstack([self.path_x, self.path_y, self.path_yaw])


        self.target_ind = self.calc_index_trajectory(self.current_x, self.current_y, self.goal_states[0, :], self.goal_states[1, :], 1)


        self.goal_flag = False
        self.hold_state = True


        ## Solver
        self.f, self.solver, self.args = NMPCSolver(
            self.lowX, self.highX, self.lowU, self.highU,
            self.mat_Q, self.mat_R, self.N, self.dt
        ).nmpc_setup()


        ## Euclidean norm condition
        self.norm_cond = 0.0

        # Optimal Control
        self.opt_u1 = 0.0
        self.opt_u2 = 0.0
        self.opt_u3 = 0.0
        self.opt_u4 = 0.0

    def forward_kinematic(self, u1, u2,u3, u4, theta):

        rot_mat = np.array([
            [np.cos(theta), np.sin(theta), 0],
            [-np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
        ], dtype=np.float64)

        J = np.array([
            [np.sin(np.pi/4), -np.sin(3*np.pi/4), np.sin(5*np.pi/4), -np.sin(7*np.pi/4)],
            [np.cos(np.pi/4), -np.cos(3*np.pi/4), np.cos(5*np.pi/4), -np.cos(7*np.pi/4)],
            [1/(2*0.23), 1/(2*0.23), 1/(2*0.23), 1/(2*0.23)]
        ], dtype=np.float64)

        for_vec = rot_mat.T@J@np.array([u1, u2, u3, u4], dtype=np.float64)

        return for_vec

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
        
        if self.hold_state:
            self.goal_states = np.tile(self.feedback_states.reshape(3, 1), self.N+1)

        self.next_trajectories[:, 0] = self.feedback_states


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


    def pi_2_pi(self, angle):
        while(angle > np.pi):
            angle = angle - 2.0 * np.pi

        while(angle < -np.pi):
            angle = angle + 2.0 * np.pi

        return angle

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


    def calc_index_trajectory(self, state_x, state_y, cx, cy, pind):

        dx = [state_x - icx for icx in cx[pind:(pind + self.N_IND_SEARCH)]]
        dy = [state_y - icy for icy in cy[pind:(pind + self.N_IND_SEARCH)]]

        d = [idx**2 + idy**2 for (idx, idy) in zip(dx, dy)]

        mind = min(d)

        ind = d.index(mind) + pind

        if pind >= ind:
            ind = pind

        return ind
     
    
    def path_callback(self, path_msg):

        self.path_msg = path_msg
        self.goal_x = path_msg.data[0]
        self.goal_y = path_msg.data[1]
        self.goal_yaw = path_msg.data[2]

        self.start_X = [self.feedback_states[0], self.feedback_states[1], self.feedback_states[2]]
        self.end_X = [self.goal_x, self.goal_y, self.goal_yaw]

        self.path, _ = self.calc_planner(self.start_X, self.end_X, self.n_points, self.offset)

        self.path_x = self.path[:, 0]
        self.path_y = self.path[:, 1]
        self.path_yaw = np.append(np.arctan2(np.diff(self.path_y), np.diff((self.path_x))), self.goal_yaw)

        self.goal_states = np.vstack([self.path_x, self.path_y, self.path_yaw])

        if path_msg.data is not None:
            self.goal_flag = True
            self.hold_state = False
        elif path_msg.data is None:
            self.goal_flag = False
            self.hold_state = True

    

    def nmpc_solver(self):
        start_time = time.time()

        if np.linalg.norm(self.goal_states[:, -1]-self.feedback_states, 2) > 0.1:
            self.norm_cond += 0.06
            self.args['lbx'][3*(self.N+1):] = -self.speed_up(self.norm_cond)
            self.args['ubx'][3*(self.N+1):] = self.speed_up(self.norm_cond)
    
        elif np.linalg.norm(self.goal_states[:, -1]-self.feedback_states, 2) < 0.01:
            self.norm_cond = 0.0
        #         self.old_nearest_index = 0
        #         self.new_nearest_index = 0
        #         self.index = 0
        #         self.dind = 0
        #         self.travel = 0.0
        else:
            self.args['lbx'][3*(self.N+1):] = self.lowU
            self.args['ubx'][3*(self.N+1):] = self.highU
            self.norm_cond = 0.0

        self.args['x0'] = np.concatenate([
            self.states.T.reshape(-1, 1),
            self.controls.T.reshape(-1, 1)
        ])

        self.args['p'] = np.concatenate([
            self.next_trajectories.T.reshape(-1, 1),
            self.next_controls.T.reshape(-1, 1)
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
        ################################################## Obtained Optimal Control ##################################################
        self.opt_u1 = sol_u.full()[0, 0]
        self.opt_u2 = sol_u.full()[1, 0]
        self.opt_u3 = sol_u.full()[2, 0]
        self.opt_u4 = sol_u.full()[3, 0]

        ################################################## Apply Next Prediction ##################################################

        self.target_ind = self.calc_index_trajectory(self.feedback_states[0], self.feedback_states[1], self.goal_states[0, :], self.goal_states[1, :], self.target_ind)

        travel = 2.0

        self.prev_index = self.next_index

        for j in range(self.N):

            self.next_trajectories[0, 0] = self.current_x
            self.next_trajectories[1, 0] = self.current_y
            self.next_trajectories[2, 0] = self.current_yaw

            # vx, vy, vyaw = self.forward_kinematic(self.feedback_controls[0], self.feedback_controls[1],
            #                                       self.feedback_controls[2], self.feedback_controls[3], 0.0)

            vx, vy, vyaw = self.forward_kinematic(self.opt_u1, self.opt_u2, self.opt_u3, self.opt_u4, 0.0)

            v = np.sqrt(vx**2+vy**2)

            travel += abs(v) * self.dt
            dind = int(round(travel / 1.0))

            self.pred_index = self.target_ind + self.index

            if (self.target_ind + self.index) < self.n_points:
                self.next_trajectories[0, j+1] = self.goal_states[0, self.pred_index]
                self.next_trajectories[1, j+1] = self.goal_states[1, self.pred_index]
                self.next_trajectories[2, j+1] = self.goal_states[2, self.pred_index]

            else:
                self.next_trajectories[0, j+1] = self.goal_states[0, self.n_points-1]
                self.next_trajectories[1, j+1] = self.goal_states[1, self.n_points-1]
                self.next_trajectories[2, j+1] = self.goal_states[2, self.n_points-1]


        self.next_index = dind
        if (self.next_index - self.prev_index) >= 2:
            self.index += 2
        
        if self.pred_index >= self.n_points:
            self.pred_index = self.n_points - 1

        x_next = self.feedback_states + self.dt * self.forward_kinematic(
             self.feedback_controls[0], self.feedback_controls[1],
             self.feedback_controls[2], self.feedback_controls[3],
             self.feedback_states[2]
        )

        self.current_states = x_next

        self.states = np.tile(self.current_states.reshape(3, 1), self.N+1)
        self.controls = np.tile(self.feedback_controls.reshape(4, 1), self.N)

        print(self.pred_index)

    def control_timer_pub(self):
        con_msg = Float32MultiArray()

        con_msg.data = [self.opt_u1, self.opt_u2, self.opt_u3, self.opt_u4]
        self.control_publisher.publish(con_msg)



def main(args=None):
    rclpy.init(args=args)

    rabbit_go = NMPCRabbit()

    rclpy.spin(rabbit_go)
    rabbit_go.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

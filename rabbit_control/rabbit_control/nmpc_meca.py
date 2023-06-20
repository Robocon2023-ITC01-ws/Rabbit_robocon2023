import rclpy
import numpy as np
import casadi as ca


from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from rabbit_control.rabbit_meca import RabbitModel
from rabbit_control.nmpc_solver import NMPCSolver
from rabbit_control.bezier_path import calc_bezier_path
from tf_transformations import euler_from_quaternion


class NMPCRabbit(Node):

    def __init__(self):

        super().__init__('nmpc_node')

        ### NMPC Params

        self.lowX = [-6, -6, -3.14]
        self.highX =[ 6,  6,  3.14]

        self.lowU = -30
        self.highU = 30

        self.Q = [1000, 1000, 2000]
        self.R = [0.1, 0.1, 0.1, 0.1]

        self.N = 50
        self.dt = 0.1

        self.rabbit_model = RabbitModel()
        self.nmpc_mode = NMPCSolver(
            self.lowX, self.highX,
            self.lowU, self.highU,
            self.Q, self.R, self.N, self.dt
        )

        ### MECANUM Params
        self.r = 0.0475
        self.lx = 0.1925
        self.ly = 0.1775

        ### Set up NMPC ###
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.feedback_states = np.array([self.current_x, self.current_y, self.current_yaw], dtype=np.float64)
        self.current_states = self.feedback_states.copy()


        self.current_w1 = 0.0
        self.current_w2 = 0.0
        self.current_w3 = 0.0
        self.current_w4 = 0.0

        self.pole_ang = 0.0
        self.pole_status = None

        self.feedback_controls = np.array([self.current_w1, self.current_w2, self.current_w3, self.current_w4], dtype=np.float64)

        self.state = np.tile(self.feedback_states.reshape(3, 1), self.N+1)
        self.control = np.tile(self.feedback_controls.reshape(4, 1), self.N)
        self.next_trajectories = np.tile(self.feedback_states.reshape(3, 1), self.N+1)
        self.next_controls = np.tile(self.feedback_controls.reshape(4, 1), self.N)

        self.f, self.solver, self.args = self.nmpc_mode.nmpc_setup()

        ### Set up trajectories ###

        self.startX = [0.0, 0.0, 0.0]
        self.endX = [0.0, 0.0, 0.0]
        self.n_points = 100
        self.offset = 1.3
        self.N_IND_SEARCH = 10

        self.path_x = np.zeros((1, self.n_points))
        self.path_y = np.zeros((1, self.n_points))
        self.path_yaw = np.zeros((1, self.n_points))

        self.goal_path = np.vstack([self.path_x, self.path_y, self.path_yaw])

        self.prev_index = 0
        self.next_index = 0
        self.index = 0
        self.target_ind = 1
        self.pred_index = self.target_ind + self.index
        self.dind = 0

        self.initial_re = True
        self.initial_g1 = True
        self.initial_g2 = True
        self.initial_g3 = True
        self.initial_g4 = True


        ### Set up params for state switches ###
        self.state_status = "manual"
        self.cmd_goal = None
        self.norm_cond = 0.0
        self.speed_up = lambda t: 30*(1-np.exp(-2*t))

        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0

        ### Set up for optimal solution ###
        self.opt_u1 = 0.0
        self.opt_u2 = 0.0
        self.opt_u3 = 0.0
        self.opt_u4 = 0.0

        self.n_u1 = 0.0
        self.n_u2 = 0.0
        self.n_u3 = 0.0
        self.n_u4 = 0.0


        ### Set up ros2 ###
        #### Subscriber ####
        self.odom_subscriber = self.create_subscription(Float32MultiArray, 'odometry_rotary', self.odom_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, 'imu/data2', self.imu_callback, 10)
        self.control_subscriber = self.create_subscription(Float32MultiArray, 'feedback_control', self.control_callback, 10)
        self.path_subscriber = self.create_subscription(Float32MultiArray, 'path_gen', self.path_callback, 10)
        self.command_subscriber = self.create_subscription(String, 'cmd_goal', self.command_callback, 10)
        self.controller_state = self.create_subscription(String, 'controller_state', self.status_callback, 10)
        self.twist_subscriber = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)
        self.pole_target = self.create_subscription(Float32, 'pole_angle', self.pole_callback, 10)
        self.pole_cmd = self.create_subscription(String, 'pole_cmd', self.pole_cmd_cb, 10)
        #### Publisher ####
        self.input_controls = self.create_publisher(Float32MultiArray, 'input_controls', 10)

        #### Timer ROS ####
        # self.control_timer = self.create_timer(0.01, self.control_timer_cb)
        self.mpc_timer = self.create_timer(0.1, self.nmpc_solver)


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


    def imu_callback(self, imu_msg):

        q1 = imu_msg.orientation.x
        q2 = imu_msg.orientation.y
        q3 = imu_msg.orientation.z
        q4 = imu_msg.orientation.w

        orient_list = [q1, q2, q3, q4]

        roll, pitch, yaw = euler_from_quaternion(orient_list)

        self.current_yaw = yaw

    def odom_callback(self, odom_msg):
        self.current_x = odom_msg.data[0]
        self.current_y = odom_msg.data[1]

        self.feedback_states = np.array([
            self.current_x, self.current_y, self.current_yaw
        ], dtype=np.float64)

        self.next_trajectories[:, 0] = self.feedback_states

        # self.state = np.tile(self.current_states.reshape(3, 1), self.N+1)


    def control_callback(self, con_msg):
        self.current_w1 = con_msg.data[0]
        self.current_w2 = con_msg.data[1]
        self.current_w3 = con_msg.data[2]
        self.current_w4 = con_msg.data[3]

        self.feedback_controls = np.array([
            self.current_w1, self.current_w2, self.current_w3, self.current_w4
        ], dtype=np.float64)

        # self.control = np.tile(self.feedback_controls.reshape(4, 1), self.N)


    def command_callback(self, cmd_msg):
        self.cmd_goal = cmd_msg.data

    def status_callback(self, status_msg):
        self.state_status = status_msg.data

    def twist_callback(self, twist_msg):
        self.vx = twist_msg.linear.x
        self.vy = twist_msg.linear.y
        self.vyaw = twist_msg.angular.z

    def pole_cmd_cb(self, msg):
        self.pole_status = msg.data

    def pole_callback(self, pole_msg):
        self.pole_ang = pole_msg.data

    def path_callback(self, path_msg):
        goal_x = path_msg.data[0]
        goal_y = path_msg.data[1]
        goal_yaw = path_msg.data[2]

        self.endX = [goal_x, goal_y, goal_yaw]

        path, _ = self.calc_planner(self.startX, self.endX, self.n_points, self.offset)

        self.path_x = path[:, 0]
        self.path_y = path[:, 1]
        self.path_yaw = np.append(np.arctan2(np.diff(self.path_y), np.diff((self.path_x))), goal_yaw)

    def nmpc_solver(self):
        if self.state_status == "manual":
            con_msg = Float32MultiArray()
            inv_vec = self.rabbit_model.inverse_kinematic(self.vx, self.vy, self.vyaw, 0.0, "numpy")

            self.n_u1 = inv_vec[0]
            self.n_u2 = inv_vec[1]
            self.n_u3 = inv_vec[2]
            self.n_u4 = inv_vec[3]

            print("IN MANUAL MODE", self.n_u1, self.n_u2, self.n_u3, self.n_u4)

            con_msg.data = [float(self.n_u1), float(self.n_u2), float(self.n_u3), float(self.n_u4)]
            self.input_controls.publish(con_msg)

            self.prev_index = 0
            self.next_index = 0
            self.index = 0
            self.pred_index = self.target_ind + self.index
            self.target_ind = 1
            self.dind = 0
            self.cmd_goal = None

            #### RESET INDEX ####
            self.goal_path = np.tile(np.array([self.current_x, self.current_y, self.current_yaw]).reshape(3, 1), self.n_points)

        elif self.state_status == "auto":
            con_msg = Float32MultiArray()

            # if self.cmd_goal == "reset":
            #     if self.initial_re:
            #         self.prev_index = 0
            #         self.next_index = 0
            #         self.index = 0
            #         self.pred_index = self.target_ind + self.index
            #         self.target_ind = 1
            #         self.dind = 0

            #         self.initial_re = False
            #         self.initial_g1 = True
            #         self.initial_g2 = True
            #         self.initial_g3 = True
            #         self.initial_g4 = True

            #     self.goal_path = np.zeros((3, self.n_points))

            #     print("Reset")

            if self.cmd_goal == "goal1":

                if self.initial_g1:
                    self.prev_index = 0
                    self.next_index = 0
                    self.index = 0
                    self.pred_index = self.target_ind + self.index
                    self.target_ind = 1
                    self.dind = 0

                    self.initial_g1 = False
                    self.initial_re = True
                    self.initial_g2 = True
                    self.initial_g3 = True
                    self.initial_g4 = True

                self.goal_path = np.vstack([self.path_x, self.path_y, self.path_yaw])

                print("Goal1")

            if self.cmd_goal == "goal2":

                if self.initial_g2:
                    self.prev_index = 0
                    self.next_index = 0
                    self.index = 0
                    self.pred_index = self.target_ind + self.index
                    self.target_ind = 1
                    self.dind = 0

                    self.initial_g2 = False
                    self.initial_re = True
                    self.initial_g1 = True
                    self.initial_g3 = True
                    self.initial_g4 = True

                self.goal_path = np.loadtxt('/home/kenotic/ros2ocp_ws/src/rabbit_control/rabbit_control/path1.csv', delimiter=',')
                print("Goal2")

            if self.cmd_goal == "goal3":

                if self.initial_g3:
                    self.prev_index = 0
                    self.next_index = 0
                    self.index = 0
                    self.pred_index = self.target_ind + self.index
                    self.target_ind = 1
                    self.dind = 0

                    self.initial_g3 = False
                    self.initial_re = True
                    self.initial_g1 = True
                    self.initial_g2 = True
                    self.initial_g4 = True
                self.goal_path = np.loadtxt('/home/kenotic/ros2ocp_ws/src/rabbit_control/rabbit_control/path2.csv', delimiter=',')

                print("Goal3")

            # if self.cmd_goal == "goal4":

            #     if self.initial_g4:
            #         self.prev_index = 0
            #         self.next_index = 0
            #         self.index = 0
            #         self.pred_index = self.target_ind + self.index
            #         self.target_ind = 1
            #         self.dind = 0

            #         self.initial_g4 = False
            #         self.initial_re = True
            #         self.initial_g1 = True
            #         self.initial_g2 = True
            #         self.initial_g3 = True
            #     self.goal_path = np.loadtxt('/home/kenotic/ros2ocp_ws/src/rabbit_control/rabbit_control/path3.csv', delimiter=',')

            #     print("Goal4")

            elif self.cmd_goal == "pole2_1":
                pole_ang = np.arctan2(self.current_y-0.0, self.current_x-3.90)
                self.goal_path = np.tile(np.array([self.current_x, self.current_y, pole_ang]).reshape(3, 1), self.n_points)
                print("Targeting Pole 2-1", self.pole_ang)
            elif self.cmd_goal == "pole2_2":
                pole_ang= np.arctan2(self.current_y-1.3, self.current_x-4.13)
                self.goal_path = np.tile(np.array([self.current_x, self.current_y, pole_ang]).reshape(3, 1), self.n_points)
                print("Targeting Pole 2-2", self.pole_ang)
            elif self.cmd_goal == "pole3":
                pole_ang= np.arctan2(self.current_y-1.3, self.current_x-4.13)
                self.goal_path = np.tile(np.array([self.current_x, self.current_y, pole_ang]).reshape(3, 1), self.n_points)
                print("Targeting Pole 3", self.pole_ang)

            if np.linalg.norm(self.goal_path[:, -1]-self.feedback_states, 2) > 0.1:
                self.norm_cond += 0.04
                self.args['lbx'][3*(self.N+1):] = -self.speed_up(self.norm_cond)
                self.args['ubx'][3*(self.N+1):] =  self.speed_up(self.norm_cond)

            elif np.linalg.norm(self.goal_path[:, -1]-self.feedback_states, 2) < 0.01:
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
                self.state.T.reshape(-1, 1),
                self.control.T.reshape(-1, 1)
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

            theta = sol_x.full()[2, 0]

            self.target_ind = self.calc_index_trajectory(self.current_states[0], self.current_states[1], self.goal_path[0, :], self.goal_path[1, :], self.target_ind)

            travel = 0.0

            self.prev_index = self.next_index

            for j in range(self.N):

                for_vec = self.rabbit_model.forward_kinematic(self.opt_u1, self.opt_u2, self.opt_u3, self.opt_u4, 0.0, "numpy")

                vx = for_vec[0]
                vy = for_vec[1]

                v = np.sqrt(vx**2+vy**2)

                travel += abs(v) * self.dt
                self.dind = int(round(travel / 1.0))

                self.pred_index = self.target_ind + self.index

                if self.pred_index >= self.goal_path.shape[1]:
                    self.pred_index = self.goal_path.shape[1]-1


                if (self.pred_index < self.goal_path.shape[1]):

                    self.next_trajectories[0, j+1] = self.goal_path[0, self.pred_index]
                    self.next_trajectories[1, j+1] = self.goal_path[1, self.pred_index]
                    self.next_trajectories[2, j+1] = self.goal_path[2, self.pred_index]

                else:

                    self.next_trajectories[0, j+1] = self.goal_path[0, self.goal_path.shape[1]-1]
                    self.next_trajectories[1, j+1] = self.goal_path[1, self.goal_path.shape[1]-1]
                    self.next_trajectories[2, j+1] = self.goal_path[2, self.goal_path.shape[1]-1]

                self.next_controls = np.tile(np.array([self.highU, self.highU, self.highU, self.highU]).reshape(4, 1), self.N)

            self.next_index = self.dind

            if (self.next_index - self.prev_index) >= 1 :
                self.index += 1

            x_next = self.feedback_states + self.rabbit_model.forward_kinematic(self.opt_u1, self.opt_u2, self.opt_u3, self.opt_u4, theta, "numpy")

            self.current_states = x_next

            self.state = np.tile(self.current_states.reshape(3, 1), self.N+1)
            self.control = np.tile(self.feedback_controls.reshape(4, 1), self.N)

            con_msg.data = [float(self.opt_u1), float(self.opt_u2), float(self.opt_u3), float(self.opt_u4)]
            self.input_controls.publish(con_msg)

            print("IN AUTO MODE", self.pred_index)
            # print(travel)
            # print(self.next_index - self.prev_index)


    # def control_timer_cb(self):
    #     if self.cmd_goal == "manual":
    #         con_msg = Float32MultiArray()
    #         con_msg.data = [float(self.n_u1), float(self.n_u2), float(self.n_u3), float(self.n_u4)]
    #         self.input_controls.publish(con_msg)
    #     if self.cmd_goal == "auto":
    #         con_msg = Float32MultiArray()
    #         con_msg.data = [float(self.opt_u1), float(self.opt_u2), float(self.opt_u3), float(self.opt_u4)]
    #         self.input_controls.publish(con_msg)



def main(args=None):

    rclpy.init(args=args)

    nmpc_node = NMPCRabbit()

    rclpy.spin(nmpc_node)

    nmpc_node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()

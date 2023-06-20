import rclpy 
import numpy as np
import casadi as ca


from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
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

        self.lowU = -20
        self.highU = 20

        self.Q = [750, 750, 900]
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
        
        self.current_w1 = 0.0
        self.current_w2 = 0.0
        self.current_w3 = 0.0
        self.current_w4 = 0.0

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
        self.offset = 3.0
        self.N_IND_SEARCH = 10

        self.path_x = np.zeros((1, self.n_points))
        self.path_y = np.zeros((1, self.n_points))
        self.path_yaw = np.zeros((1, self.n_points))

        self.goal_path = np.vstack([self.path_x, self.path_y, self.path_yaw])

        ### Set up params for state switches ###
        self.state_status = None
        self.cmd_goal = None
        self.norm_cond = 0.0
        self.speed_up = lambda t: 20*(1-np.exp(-2*t))

        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0

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
        self.odom_subscriber = self.create_subscription(Float32MultiArray, 'odom_wheel', self.odom_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, 'imu/data2', self.imu_callback, 10)
        self.control_subscriber = self.create_subscription(Float32MultiArray, 'feedback_control', self.control_callback, 10)
        self.path_subscriber = self.create_subscription(Float32MultiArray, 'beizer_path', self.path_callback, 10)
        self.command_subscriber = self.create_subscription(String, 'cmd_goal', self.command_callback, 10)
        self.controller_state = self.create_subscription(String, 'controller_state', self.status_callback, 10)
        self.twist_subscriber = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)
        #### Publisher ####
        self.input_controls = self.create_publisher(Float32MultiArray, 'input_controls', 10)

        #### Timer ROS ####
        # self.control_timer = self.create_timer(0.01, self.control_timer_cb)
        self.mpc_timer = self.create_timer(0.1, self.nmpc_solver)



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

        self.state = np.tile(self.feedback_states.reshape(3, 1), self.N+1)

    def control_callback(self, con_msg):
        self.current_w1 = con_msg.data[0]
        self.current_w2 = con_msg.data[1]
        self.current_w3 = con_msg.data[2]
        self.current_w4 = con_msg.data[3]

        self.feedback_controls = np.array([
            self.current_w1, self.current_w2, self.current_w3, self.current_w4
        ], dtype=np.float64)

        self.control = np.tile(self.feedback_controls.reshape(4, 1), self.N)

    def command_callback(self, cmd_msg):
        self.cmd_goal = cmd_msg.data

    def status_callback(self, status_msg):
        self.state_status = status_msg.data

    def twist_callback(self, twist_msg):
        self.vx = twist_msg.linear.x
        self.vy = twist_msg.linear.y
        self.vyaw = twist_msg.angular.z

    def path_callback(self, path_msg):
        self.goal_x = path_msg.data[0]
        self.goal_y = path_msg.data[1]
        self.goal_yaw = path_msg.data[2]

    
    def nmpc_solver(self):
        # if self.cmd_goal == "reset":

        #     self.goal_path = np.zeros((3, self.n_points))
        
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

        elif self.state_status == "auto":

            con_msg = Float32MultiArray()

            if np.linalg.norm(self.goal_path[:, -1]-self.feedback_states, 2) > 0.1:
                self.norm_cond += 0.06
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


            for j in range(self.N):

                self.next_trajectories[0, j+1] = self.goal_x
                self.next_trajectories[1, j+1] = self.goal_y
                self.next_trajectories[2, j+1] = self.goal_yaw

                self.next_controls = np.tile(np.array([self.highU, self.highU, self.highU, self.highU]).reshape(4, 1), self.N)

            con_msg.data = [float(self.opt_u1), float(self.opt_u2), float(self.opt_u3), float(self.opt_u4)]

            self.input_controls.publish(con_msg)
        





def main(args=None):

    rclpy.init(args=args)

    nmpc_node = NMPCRabbit()

    rclpy.spin(nmpc_node)

    nmpc_node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
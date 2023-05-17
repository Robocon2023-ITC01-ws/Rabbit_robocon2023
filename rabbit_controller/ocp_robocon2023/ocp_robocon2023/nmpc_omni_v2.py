import rclpy
import numpy as np
import casadi as ca

from .library.casadi_solver_rabbit import CasadiNMPC
from .library.rabbit_omni import RabbitModel
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int64, Bool
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

        self.lowU = [-15, -15, -15 , -15]
        self.highU = [15, 15, 15, 15]

        # self.lowU = [-20, -20, -20, -20]
        # self.highU = [20, 20, 20, 20]

        self.mat_Q = [750, 750, 2000]
        self.mat_R = [1, 1, 1, 1]

        self.goal_flag = False

        self.mpc_type = "circle"
        self.index = 0
        self.N = 50
        self.dt = 0.05
        self.mpciter = 0
        self.speed_up = lambda t: 20*(1-np.exp(-2*t))
        self.n_points = 200

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
                                    ], dtype=np.float32)
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
            N=self.N, dt=self.dt, sim_time = 23
        )

        self.goal_states = np.zeros(3)

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
        self.odom_subscriber = self.create_subscription(Float32MultiArray, 'state_est', self.odom_callback, 10)
        self.control_subscriber = self.create_subscription(Float32MultiArray, 'feedback_encoder', self.controls_callback, 10)
        self.quaternion_subscriber = self.create_subscription(Imu, 'imu/data2', self.quaternion_callback, 10)
        self.control_publisher = self.create_publisher(Float32MultiArray, 'input_controls', 10)
        self.path_gen = self.create_subscription(Float32MultiArray, 'beizer_path', self.path_callback, 10)
        self.control_timer = self.create_timer(0.01, self.control_timer_pub)
        self.solver_time = self.create_timer(0.1, self.nmpc_solver)


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
        

    def controls_callback(self, con_msg):
        
        for_vec_tran = self.rabbit_model.forward_kinematic_tran(con_msg.data[0], con_msg.data[1],
                                    con_msg.data[2], con_msg.data[3], 0.0, "numpy")
        
        inv_vec = self.rabbit_model.inverse_kinematic(for_vec_tran[0], for_vec_tran[1], for_vec_tran[2], 0.0, "numpy")

        self.current_w1 = inv_vec[0]
        self.current_w2 = inv_vec[1]
        self.current_w3 = inv_vec[2]
        self.current_w4 = inv_vec[3]

        self.feedback_controls = np.array([
                                    self.current_w1,
                                    self.current_w2,
                                    self.current_w3,
                                    self.current_w4
                                    ])

    def path_callback(self, path_msg):

        goal_x = path_msg.data[0]
        goal_y = path_msg.data[1]
        goal_yaw = path_msg.data[2]

        self.goal_states = np.array([goal_x, goal_y, goal_yaw])


    def nmpc_solver(self):
        start_time = self.get_clock().now()
        if np.linalg.norm(self.goal_states-self.current_states, 2) > 0.01:
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
        ###### Saturation
        if self.opt_u1 > self.highU or self.opt_u2 > self.highU or self.opt_u3 > self.highU or self.opt_u4 > self.highU:
            self.opt_u1 = self.highU
            self.opt_u2 = self.highU
            self.opt_u3 = self.highU
            self.opt_u4 = self.highU
        elif self.opt_u1 < self.lowU or self.opt_u2 < self.lowU or self.opt_u3 < self.lowU or self.opt_u4 < self.lowU:
            self.opt_u1 = self.lowU
            self.opt_u2 = self.lowU
            self.opt_u3 = self.lowU
            self.opt_u4 = self.lowU
        ##############################################################################################################################
        ################################################## Shift Timestep ##################################################

        self.current_states = ca.DM.full(self.feedback_states.reshape(3, 1) + self.dt * self.f(self.feedback_states, self.feedback_controls))

        self.states = np.tile(self.current_states.reshape(3, 1), self.N+1)
        self.controls = np.tile(self.feedback_controls.reshape(4, 1), self.N)
        ################################################## Apply Next Prediction ##################################################
        
        for k in range(self.N):
            self.next_trajectories[0, 0] = self.current_states[0]
            self.next_trajectories[1, 0] = self.current_states[1]
            self.next_trajectories[2, 0] = self.current_states[2]
            self.next_trajectories[0, k+1] = self.goal_states[0]
            self.next_trajectories[1, k+1] = self.goal_states[1]
            self.next_trajectories[2, k+1] = self.goal_states[2]

            self.next_controls = np.tile(np.array([15, 15, 15, 15], dtype=np.float32), self.N)

            if (np.linalg.norm(self.current_states-self.goal_states, 2) > 0.4):
                self.next_controls = np.tile(np.array([15, 15, 15, 15], dtype=np.float64).reshape(4, 1), self.N)
            elif ((np.linalg.norm(self.current_states-self.goal_states, 2) < 0.01)):
                self.next_controls = np.tile(np.array([0, 0, 0, 0], dtype=np.float64).reshape(4, 1), self.N)
        

        ############################################################################################################################
        end_time = self.get_clock().now()

        duration = (end_time - start_time )

    def control_timer_pub(self):
        con_msg = Float32MultiArray()
        
        for_vec = self.rabbit_model.forward_kinematic(self.opt_u1, self.opt_u2, self.opt_u3, self.opt_u4, 0.0, "numpy")

        inv_vec_tran = self.rabbit_model.inverse_kinematic_tran(for_vec[0], for_vec[1], for_vec[2], 0.0, "numpy")

        con_msg.data = [inv_vec_tran[0], inv_vec_tran[1], inv_vec_tran[2], inv_vec_tran[3]]
        self.control_publisher.publish(con_msg)

        self.get_logger().info("Publishing optimal control")



def main(args=None):
    rclpy.init(args=args)

    rabbit_go = NMPCRabbit()

    rclpy.spin(rabbit_go)
    rabbit_go.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

import rclpy
import numpy as np
import casadi as ca
import time

from .library.casadi_solver_rabbit import CasadiNMPC
from .library.rabbit_omni import RabbitModel
from .library.bezier_path import calc_4points_bezier_path, calc_bezier_path

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from mpc_action.action import MPCAction
from std_msgs.msg import Float32MultiArray
from tf_transformations import euler_from_quaternion

class MPCActionServer(Node):

    def __init__(self):
        super().__init__('mpc_server_node')


        ## ROS SETUP
        self.odom_subscriber = self.create_subscription(Float32MultiArray, 'state_est', self.odom_callback, 100)
        self.control_subscirber = self.create_subscription(Float32MultiArray, 'feedback_encoders', self.controls_callback, 100)
        self.control_publisher = self.create_publisher(Float32MultiArray, 'input_controls', 100)
        self.control_timer = self.create_timer(0.01, self.control_timer_pub)
        # ROS ACTION
        self.goal_handle = None
        self.mpc_server = ActionServer(
                self,
                MPCAction,
                'mpc_action',
                execute_callback=self.mpc_callback,
                callback_group=ReentrantCallbackGroup(),
                handle_accepted_callback=self.handle_accepted_callback,
                cancel_callback=self.cancel_callback)
        self.initX = [0.0, 0.0, 0.0]
        self.initU = [0.0, 0.0, 0.0, 0.0]

        self.lowX =  [-6, -6, -3.14]
        self.highX = [ 6,  6, 3.14]

        self.lowU = [-20, -20, -20, -20]
        self.highU = [20, 20, 20, 20]

        self.mat_Q = [750, 750, 2000]
        self.mat_R = [1, 1, 1, 1]

        self.mpc_type = "circle"
        self.index = 0
        self.N = 50
        self.dt = 0.01
        self.t0 = 0
        self.mpciter = 0
        self.sim_time = 23
        self.speed_up = lambda t: 20*(1-np.exp(-2*t))

        ## States
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
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
        self.opt_u1 = 0.0
        self.opt_u2 = 0.0
        self.opt_u3 = 0.0
        self.opt_u4 = 0.0
        self.feedback_controls = np.array([
                                        self.current_w1,
                                        self.current_w2,
                                        self.current_w3,
                                        self.current_w4])

        self.init_states = self.feedback_states.copy()
        self.states = np.tile(self.feedback_states.reshape(3, 1), self.N+1)
        self.controls = np.tile(self.feedback_controls.reshape(4, 1), self.N)
        self.next_trajectories = self.states.copy()
        self.next_controls = self.controls.copy()
        self.casadi_solver = CasadiNMPC(
                            initX=self.init_states, initU=self.feedback_controls[0],
                            lowX=self.lowX, highX=self.highX,
                            lowU=self.lowU, highU=self.highU,
                            mat_Q=self.mat_Q, mat_R=self.mat_R,
                            N=self.N, dt=self.dt, sim_time=self.sim_time)
        
        self.n_points = 100
        self.offset = 2.0

        ## Solver
        self.f, self.solver, self.args = self.casadi_solver.casadi_model_setup()
        self.rabbit_model = RabbitModel()

        ## Euclidean norm condition
        self.norm_cond = 0.0

    def destroy(self):
        self.mpc_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT


    def handle_accepted_callback(self, goal_handle):
        self.get_logger().info('Deferring execution')
        self.goal_handle = goal_handle
        self.timer = self.create_timer(2.0, self.timer_callback)


    def timer_callback(self):
        if self.goal_handle is not None:
            self.goal_handle.execute()
        self.timer.cancel()
    
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    

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

        return np.vstack([path_x, path_y, path_yaw])
    

    async def mpc_callback(self, goal_handle):
        self.get_logger().info('Executing task...')

        goal_feedback = MPCAction.Feedback()
        control_result = MPCAction.Result()

        goal = np.array(goal_handle.request.goal_states)
        # print(goal)
        goal_states = self.path_beizer(goal)
        while (np.linalg.norm(goal_states[:, -1] - self.current_states, 2) > 0.01):
            if np.linalg.norm(goal_states[:, -1] - self.current_states, 2) > 0.3:
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
            ################################################## Obtained Optimal Control ##################################################
            self.opt_u1 = sol_u.full()[0, self.index]
            self.opt_u2 = sol_u.full()[1, self.index]
            self.opt_u3 = sol_u.full()[2, self.index]
            self.opt_u4 = sol_u.full()[3, self.index]
            ##############################################################################################################################
            ################################################## Shift Timestep ##################################################

            self.current_states = ca.DM.full(self.feedback_states.reshape(3, 1) + self.dt * self.f(self.feedback_states, self.feedback_controls))

            self.states = np.tile(self.current_states.reshape(3, 1), self.N+1)
            self.controls = np.tile(self.feedback_controls.reshape(4, 1), self.N)
            ################################################## Apply Next Prediction ##################################################
            
            for k in range(self.N):
                index = self.mpciter + k

                if index >= goal_states.shape[1]:
                    index = goal_states.shape[1]-1
                
                self.next_trajectories[0, 0] = self.current_states[0]
                self.next_trajectories[1, 0] = self.current_states[1]
                self.next_trajectories[2, 0] = self.current_states[2]
                self.next_trajectories[0, k+1] = goal_states[0, index]
                self.next_trajectories[1, k+1] = goal_states[1, index]
                self.next_trajectories[2, k+1] = goal_states[2, index]

                self.next_controls = np.tile(np.array([15, 15, 15, 15], dtype=np.float32), self.N)

                if (np.linalg.norm(self.current_states-goal_states[:, -1], 2) > 0.4):
                    self.next_controls = np.tile(np.array([20, 20, 20, 20], dtype=np.float64).reshape(4, 1), self.N)
                elif ((np.linalg.norm(self.current_states-goal_states[:, -1], 2) < 0.01)):
                    self.next_controls = np.tile(np.array([0, 0, 0, 0], dtype=np.float64).reshape(4, 1), self.N)

            goal_feedback.goal_feedback = [self.current_x, self.current_y, self.current_yaw]
            goal_handle.publish_feedback(goal_feedback)
            
            # print(self.test_num)current_vy, self.current_vth = self.rabbit_model.forward_kinematic(
                              
            # print(self.mpciter)
            self.mpciter += 1
            time.sleep(0.1)

        if (np.linalg.norm(goal_states[goal_states.shape[0]-1] - self.current_states, 2) == 0.01):
            goal_handle.succeed()

        result = MPCAction.Result()
        result.goal_results = goal_feedback.goal_feedback


        return result
    
    def control_timer_pub(self):
        con_msg = Float32MultiArray()
        con_msg.data = [self.opt_u1, self.opt_u2, self.opt_u3, self.opt_u4]
        self.control_publisher.publish(con_msg)

        # self.get_logger().info("Publishing optimal control '%s'" % con_msg)

        


def main(args=None):
    rclpy.init(args=args)

    mpc_server = MPCActionServer()

    executor = MultiThreadedExecutor()

    rclpy.spin(mpc_server, executor=executor)

    mpc_server.destroy()

    rclpy.shutdown()


if __name__=='__main__':
    main()

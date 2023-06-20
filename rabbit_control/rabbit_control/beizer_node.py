import rclpy
import numpy as np

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import Imu, Joy
from rabbit_control.bezier_path import calc_bezier_path
from rabbit_control.rabbit_meca import RabbitModel
from tf_transformations import euler_from_quaternion
import time


class TrajectoryGenerator(Node):

    def __init__(self):

        super().__init__('beizer_node')

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.n_points = 100

        self.feedback_states = np.array([self.current_x, self.current_y, self.current_yaw])
        self.current_states = self.feedback_states.copy()
        self.startX = [0.0, 0.0, 0.0]

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0

        self.opt_u1 = 0
        self.opt_u2 = 0
        self.opt_u3 = 0
        self.opt_u4 = 0

        self.endX = [self.goal_x, self.goal_y, self.goal_yaw]

        self.path = np.zeros((self.n_points, 2))
        self.offset = 1.0
        self.N = 50
        self.dt = 0.1
        self.prev_index = 0
        self.next_index = 0
        self.N_IND_SEARCH = 10
        self.target_ind = 0
        self.index = 0
        self.pred_index = self.index + self.target_ind

        self.status_state = None

        self.rabbit_model = RabbitModel()

        self.path_x = np.tile(self.current_x, self.n_points)
        self.path_y = np.tile(self.current_y, self.n_points)
        self.path_yaw = np.tile(self.current_yaw, self.n_points)

        self.goal_states = np.vstack([self.path_x, self.path_y, self.path_yaw])

        self.target_ind = self.calc_index_trajectory(self.current_x, self.current_y, self.goal_states[0], self.goal_states[1], 1)

        self.odom_subscriber = self.create_subscription(Float32MultiArray, 'odom_wheel', self.odom_callback, 10)
        self.input_subscriber = self.create_subscription(Float32MultiArray, 'input_controls', self.controls_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, 'imu/data2', self.imu_callback, 10)
        self.goal_subscriber = self.create_subscription(Float32MultiArray, 'path_gen', self.goal_callback, 10)
        self.control_subscriber = self.create_subscription(Float32MultiArray, 'feedback_controls', self.controls_callback, 10)
        self.goal_cmd = self.create_subscription(String, 'cmd_goal', self.cmd_goal_callback, 10)
        self.status_subscriber = self.create_subscription(String, 'controller_states', self.states_callback, 10)

        self.path_publisher = self.create_publisher(Float32MultiArray, 'beizer_path', 10)
        self.joy_subscriber = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.path_timer = self.create_timer(1/10, self.path_callback)

        self.axes_list = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.buttons_list = [0, 0, 0, 0, 0, 0, 0, 0, 0]



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
    
    def imu_callback(self, quat_msg):
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
            self.current_yaw
        ])

        # self.startX = [self.current_x, self.current_y, self.current_yaw]

    def states_callback(self, status_msg):
        self.status_state = status_msg.data

    def calc_index_trajectory(self, state_x, state_y, cx, cy, pind):
        
        dx = [state_x - icx for icx in cx[pind:(pind + self.N_IND_SEARCH)]]
        dy = [state_y - icy for icy in cy[pind:(pind + self.N_IND_SEARCH)]]

        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

        mind = min(d)

        ind = d.index(mind) + pind

        return ind
        

    def joy_callback(self, joy_msg):

        self.axes_list = joy_msg.axes
        self.buttons_list = joy_msg.buttons

        if self.buttons_list[2] == 1:
            self.start_cond = True
        elif self.buttons_list[2] == 0:
            self.start_cond = False


    def controls_callback(self, con_msg):
        self.opt_u1 = con_msg.data[0]
        self.opt_u2 = con_msg.data[1]
        self.opt_u3 = con_msg.data[2]
        self.opt_u4 = con_msg.data[3]


    def goal_callback(self, goal_msg):
        self.goal_x = goal_msg.data[0]
        self.goal_y = goal_msg.data[1]
        self.goal_yaw = goal_msg.data[2]

        self.endX = [self.goal_x, self.goal_y, self.goal_yaw]


        self.path, _ = self.calc_planner(
             self.startX, self.endX, self.n_points, self.offset
        )

        self.path_x = self.path[:, 0]
        self.path_y = self.path[:, 1]
        self.path_yaw = np.append(np.arctan2(np.diff(self.path_y), np.diff((self.path_x))), self.goal_yaw)

        self.goal_states = np.vstack([self.path_x, self.path_y, self.path_yaw])

    def cmd_goal_callback(self, cmd_msg):

        if cmd_msg.data == "reset":

            self.goal_states = np.zeros((3, self.n_points))

        if cmd_msg.data == "goal1":
            self.startX = [0, 0, 0]
            self.endX = [2.7, 1.5, 1.57]
            self.path, _ = self.calc_planner(
                self.startX, self.endX, self.n_points, -3.0
            )

            self.path_x = self.path[:, 0]
            self.path_y = self.path[:, 1]
            self.path_yaw = np.linspace(0, 1.57, self.n_points)

            self.goal_states = np.vstack([self.path_x, self.path_y, self.path_yaw])

        elif cmd_msg.data == "goal2":
            self.startX = [2.7, 1.5, 1.57]
            self.endX = [0.0, 0.0, 0.0]
            self.path, _ = self.calc_planner(
                self.startX, self.endX, self.n_points, -3.0
            )

            self.path_x = self.path[:, 0]
            self.path_y = self.path[:, 1]
            self.path_yaw = np.linspace(1.57, 0, self.n_points)

            self.goal_states = np.vstack([self.path_x, self.path_y, self.path_yaw])

        # print(self.goal_states[:, 2])
    
    def path_callback(self):
        start = time.time()
        path_msg = Float32MultiArray()

        self.target_ind = self.calc_index_trajectory(self.current_states[0], self.current_states[1], self.goal_states[0, :], self.goal_states[1, :], self.target_ind)

        travel = 1.0

        self.prev_index = self.next_index

        for k in range(self.N):

            for_vec = self.rabbit_model.forward_kinematic(self.opt_u1, self.opt_u2, self.opt_u3, self.opt_u4, 0.0, "numpy")

            vx = for_vec[0]
            vy = for_vec[1]

            v = np.sqrt(vx**2+vy**2)

            travel += abs(v) * self.dt
            self.dind = int(round(travel / 1.0))

            self.pred_index = self.target_ind + self.index

            if self.pred_index >= self.goal_states.shape[1]:
                self.pred_index = self.goal_states.shape[1]-1

            if (self.target_ind + self.index) < len(self.path_x):
                path_msg.data = [float(self.goal_states[0, self.pred_index]),
                                 float(self.goal_states[1, self.pred_index]),
                                 float(self.goal_states[2, self.pred_index])]
            
            else:
                path_msg.data = [float(self.goal_states[0, len(self.path_x)-1]),
                                 float(self.goal_states[1, len(self.path_x)-1]),
                                 float(self.goal_states[2, len(self.path_x)-1])]
            
            self.path_publisher.publish(path_msg)

        self.next_index = self.dind
        if (self.next_index-self.prev_index) >= 2:
            self.index += 3
        
        self.current_states = self.feedback_states + self.rabbit_model.forward_kinematic(self.opt_u1, self.opt_u2, self.opt_u3, self.opt_u4, self.current_yaw, "numpy") * self.dt
    

        print(self.pred_index)


def main(args=None):
    rclpy.init(args=args)

    node = TrajectoryGenerator()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__== "__main__":
    main()

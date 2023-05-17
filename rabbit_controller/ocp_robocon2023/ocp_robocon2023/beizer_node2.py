import rclpy
import numpy as np

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from sensor_msgs.msg import Imu, Joy
from .library.bezier_path import calc_bezier_path, calc_4points_bezier_path
from tf_transformations import euler_from_quaternion


class TrajectoryGenerator(Node):

    def __init__(self):

        super().__init__('beizer_node')

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.current_states = np.array([self.current_x, self.current_y, self.current_yaw])
        self.startX = [self.current_x, self.current_y, self.current_yaw]

        self.n_points = 200

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0 

        self.endX = [self.goal_x, self.goal_y, self.goal_yaw]

        self.path = np.zeros((self.n_points, 2))
        self.offset = 5.0

        self.goal_states = np.zeros((3, self.n_points))

        self.goal_cond = False

        self.path_x = np.zeros((self.n_points, 1))
        self.path_y = np.zeros((self.n_points, 1))
        self.path_yaw = np.zeros((self.n_points, 1))

        self.index = 0

        self.odom_subscriber = self.create_subscription(Float32MultiArray, 'state_est', self.odom_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, 'imu/data2', self.imu_callback, 10)
        self.goal_subscriber = self.create_subscription(Joy, 'joy', self.goal_callback, 10)

        self.path_publisher = self.create_publisher(Float32MultiArray, 'beizer_path', 10)

        self.path_timer = self.create_timer(1/10, self.path_callback)

        self.axes_list = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.button_list = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


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

        self.current_states = np.array([
            self.current_x,
            self.current_y,
            self.current_yaw
        ])

        self.startX = [self.current_x, self.current_y, self.current_yaw]


    def odom_callback(self, odom_msg):
        self.current_x = odom_msg.data[0]
        self.current_y = odom_msg.data[1]

        self.current_states = np.array([
            self.current_x,
            self.current_y,
            self.current_yaw
        ])


    def goal_callback(self, joy):

        self.axes_list = joy.axes
        self.button_list = joy.buttons

        self.startX = [self.current_x, self.current_y, self.current_yaw]
        self.endX = [3.0, 0.0, 0.0]

        if self.button_list[3] == 1:
            self.goal_cond = True

        elif self.button_list[3] != 1:
            self.goal_cond = False
            print('Still in the current states')

    
    def path_callback(self):
        path_msg = Float32MultiArray()

        if self.goal_cond:
            print('Calculating Path')
            if self.axes_list[7] == 1:
                self.goal_states = np.loadtxt('/home/kenotic/ros2ocp_ws/src/ocp_robocon2023/ocp_robocon2023/library/path7.csv', delimiter=',', dtype=np.float32)
            #if self.axes_list[6] == 1:
             #   self.goal_states = np.loadtxt('/home/kenotic/ros2ocp_ws/src/ocp_robocon2023/ocp_robocon2023/library/path3.csv', delimiter=',', dtype=np.float32)
        elif not self.goal_cond:
            self.goal_states = np.tile(self.current_states.reshape(3, 1), self.n_points)
        # if np.linalg.norm(self.current_states-self.goal_states[:, -1], 2) > 0.2:
        #     self.index += 2
        #     if self.index >= self.n_points:
        #         self.index = self.n_points - 1
        # if np.linalg.norm(self.current_states-self.goal_states[:, -1], 2) < 0.001:
        #     self.index = 0 
        if self.goal_cond:
            self.index += 2
            if self.index >= self.goal_states.shape[1]:
                self.index = self.goal_states.shape[1]-1
        if self.button_list[1] == 1:
            self.index = 0

        path_msg.data = [float(self.goal_states[0, self.index]), float(self.goal_states[1, self.index]), float(self.goal_states[2, self.index])]
        self.path_publisher.publish(path_msg)

        print(self.index)



def main(args=None):
    rclpy.init(args=args)

    node = TrajectoryGenerator()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__== "__main__":
    main()

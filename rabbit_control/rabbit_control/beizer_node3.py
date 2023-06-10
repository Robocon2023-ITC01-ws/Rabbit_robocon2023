import rclpy
import numpy as np

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool, String
from sensor_msgs.msg import Imu, Joy
from rabbit_control.rabbit_meca import RabbitModel
from rabbit_control.bezier_path import calc_bezier_path, calc_4points_bezier_path
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

        self.N = 50

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0 

        self.endX = [self.goal_x, self.goal_y, self.goal_yaw]

        self.path = np.zeros((self.n_points, 2))
        self.dt = 0.1

        self.goal_states = np.zeros((3, self.n_points))

        self.goal_cond = False
        self.goal_msg = None

        self.path_x = np.zeros((self.n_points, 1))
        self.path_y = np.zeros((self.n_points, 1))
        self.path_yaw = np.zeros((self.n_points, 1))

        self.N_IND_SEARCH = 10

        self.rabbit_model = RabbitModel()

        self.odom_subscriber = self.create_subscription(Float32MultiArray, 'state_est', self.odom_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, 'imu/data2', self.imu_callback, 10)
        self.goal_subscriber = self.create_subscription(Joy, 'joy', self.goal_callback, 10)
        self.command_goal = self.create_subscription(String, "controller_state", self.command_callback, 10)

        self.path_publisher = self.create_publisher(Float32MultiArray, 'beizer_path', 10)
        self.input_subscriber = self.create_subscription(Float32MultiArray, 'input_controls', self.input_callback, 10)

        self.path_timer = self.create_timer(1/10, self.path_callback)

        self.axes_list = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.button_list = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.opt_u = [0.0, 0.0, 0.0, 0.0]


        ################### INDEX ###################
        self.p_ind1 = 0
        self.c_ind1 = 0
        self.pre_ind1 = 0
        self.t_ind1 = 2
        self.ind1 = 0
        ################### INDEX ###################
        self.p_ind2 = 0
        self.c_ind2 = 0
        self.pre_ind2 = 0
        self.t_ind2 = 2
        self.ind2 = 0
        ################### INDEX ###################
        self.p_ind3 = 0
        self.c_ind3 = 0
        self.pre_ind3 = 0
        self.t_ind3 = 2
        self.ind3 = 0


    def command_callback(self, msg):

        self.goal_msg = msg.data

    def odom_callback(self, odom_msg):
        self.current_x = odom_msg.data[0]
        self.current_y = odom_msg.data[1]

        self.current_states = np.array([
            self.current_x,
            self.current_y,
            self.current_yaw
        ])

    def imu_callback(self, quat_msg):
        q1 = quat_msg.orientation.x
        q2 = quat_msg.orientation.y
        q3 = quat_msg.orientation.z
        q4 = quat_msg.orientation.w

        orient_list = [q1, q2, q3, q4]

        roll, pitch, yaw = euler_from_quaternion(orient_list)

        self.current_yaw = yaw

    def input_callback(self, con_msg):

        self.opt_u[0] = con_msg.data[0]
        self.opt_u[1] = con_msg.data[1]
        self.opt_u[2] = con_msg.data[2]
        self.opt_u[3] = con_msg.data[3]



    def calc_index_trajectory(self, state_x, state_y, cx, cy, pind):

        dx = [state_x - icx for icx in cx[pind:(pind + self.N_IND_SEARCH)]]
        dy = [state_y - icy for icy in cy[pind:(pind + self.N_IND_SEARCH)]]

        d = [idx**2 + idy**2 for (idx, idy) in zip(dx, dy)]

        mind = min(d)

        ind = d.index(mind) + pind

        if pind >= ind:
            ind = pind

        return ind


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

        if self.goal_msg == "goal1":
            self.goal_states = np.loadtxt('/home/kenotic/ros2ocp_ws/src/ocp_robocon2023/ocp_robocon2023/library/path7.csv', delimiter=',', dtype=np.float32)
            
            self.ind1 = self.calc_index_trajectory(self.current_x, self.current_y,
                                                          self.goal_states[0], self.goal_states[1],
                                                          self.ind1)
            travel = 2.0

            self.p_ind1 = self.c_ind1

            for j in range(self.N):

                for_vec = self.rabbit_model.forward_kinematic(self.opt_u[0], self.opt_u[1],
                                                              self.opt_u[2], self.opt_u[3], 0.0, "numpy")
                vx = for_vec[0]
                vy = for_vec[1]
                
                v = np.sqrt(vx**2+vy**2)

                travel += abs(v) * self.dt 
                dind = int(round(travel / 1.0))

                self.pre_ind1 = self.ind1 + self.t_ind1

                if (self.t_ind1 + self.ind1) < self.goal_states.shape[1]:

                    path_msg.data = [float(self.goal_states[0, self.pre_ind1]),
                                     float(self.goal_states[1, self.pre_ind1]),
                                     float(self.goal_states[2, self.pre_ind1])]
                else:
                    path_msg.data = [float(self.goal_states[0, self.goal_states.shape[1]-1]),
                                     float(self.goal_states[1, self.goal_states.shape[1]-1]),
                                     float(self.goal_states[2, self.goal_states.shape[1]-1])]
            
            self.c_ind1 = dind
            if (self.c_ind1 - self.p_ind1) >= 2:
                self.ind1 += 3
            
            if self.pre_ind1 >= self.goal_states.shape[1]:
                self.pre_ind1 = self.goal_states.shape[1]-1

                    
        elif self.goal_msg == "goal2":
            self.goal_states = np.loadtxt('/home/kenotic/ros2ocp_ws/src/ocp_robocon2023/ocp_robocon2023/library/path7.csv', delimiter=',', dtype=np.float32)
            
            self.ind2 = self.calc_index_trajectory(self.current_x, self.current_y,
                                                   self.goal_states[0], self.goal_states[1],
                                                   self.ind2)
            travel = 2.0

            self.p_ind2 = self.c_ind2

            for j in range(self.N):

                for_vec = self.rabbit_model.forward_kinematic(self.opt_u[0], self.opt_u[1],
                                                              self.opt_u[2], self.opt_u[3], 0.0, "numpy")
                vx = for_vec[0]
                vy = for_vec[1]
                
                v = np.sqrt(vx**2+vy**2)

                travel += abs(v) * self.dt 
                dind = int(round(travel / 1.0))

                self.pre_ind2 = self.ind2 + self.t_ind2

                if (self.t_ind2 + self.ind2) < self.goal_states.shape[1]:

                    path_msg.data = [float(self.goal_states[0, self.pre_ind2]),
                                     float(self.goal_states[1, self.pre_ind2]),
                                     float(self.goal_states[2, self.pre_ind2])]
                else:
                    path_msg.data = [float(self.goal_states[0, self.goal_states.shape[1]-1]),
                                     float(self.goal_states[1, self.goal_states.shape[1]-1]),
                                     float(self.goal_states[2, self.goal_states.shape[1]-1])]
            
            self.c_ind2 = dind
            if (self.c_ind2 - self.p_ind2) >= 2:
                self.ind2 += 3
            
            if self.pre_ind2 >= self.goal_states.shape[1]:
                self.pre_ind2 = self.goal_states.shape[1]-1



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

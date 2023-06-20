import numpy as np
import casadi as ca
import rclpy

from tf_transformations import euler_from_quaternion
from rabbit_localization.nmhe_solver import NMHESolver
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, UInt16MultiArray
from sensor_msgs.msg import Imu


class NMHE_Rabbit(Node):

    def __init__(self):

        super().__init__('nmhe_node')


        self.lowX = [-6, -6, -3.14]
        self.highX = [6,  6,  3.14]

        self.lowU = -50
        self.highU = 50

        self.V = np.sqrt(np.diag([0.01, 0.01, 0.1]))
        self.W = np.sqrt(np.diag([0.01, 0.01, 0.01, 0.01]))

        self.N_mhe = 10

        self.nmhe_setup = NMHESolver(
            self.lowX, self.highX, self.lowU, self.highU,
            self.V, self.W, self.N_mhe
        )

        self.f, self.solver, self.args = self.nmhe_setup.nmhe_solver()

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.mhe_x = 0.0
        self.mhe_y = 0.0


        self.feedback_states = np.array([
            self.current_x,
            self.current_y,
            self.current_yaw
        ], dtype=np.float64)

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

        self.states = np.tile(self.feedback_states.reshape(3, 1), self.N_mhe+1)
        self.controls = np.tile(self.feedback_controls.reshape(4, 1), self.N_mhe)

        self.meas_states = np.tile(self.feedback_states.reshape(3, 1), self.N_mhe+1)
        self.meas_controls = np.tile(self.feedback_controls.reshape(4, 1), self.N_mhe)


        self.odom_subscriber = self.create_subscription(Float32MultiArray, 'odometry_rotary' , self.odom_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, 'imu/data2', self.imu_callback, 10)
        self.control_subscriber = self.create_subscription(Float32MultiArray, 'feedback_controls', self.control_callback, 10)
        # self.input_rotary = self.create_subscription(UInt16MultiArray, 'external_rotary', self.rotary_callback, 10)


        self.feedback_publisher = self.create_publisher(Float32MultiArray, 'odom_mhe', 10)

        self.mhe_timer = self.create_timer(1/50, self.mhe_callback)

    def imu_callback(self, imu_msg):

        q1 = imu_msg.orientation.x
        q2 = imu_msg.orientation.y
        q3 = imu_msg.orientation.z
        q4 = imu_msg.orientation.w

        orient_list = [q1, q2, q3, q4]

        roll, pitch, yaw = euler_from_quaternion(orient_list)

        self.current_yaw = yaw

        self.feedback_states = np.array([
            self.current_x,
            self.current_y,
            self.curren_yaw
        ], dtype=np.float64)

    def control_callback(self, con_msg):

        self.current_w1 = con_msg.data[0]
        self.current_w2 = con_msg.data[1]
        self.current_w3 = con_msg.data[2]
        self.current_w4 = con_msg.data[3]

        self.feedback_controls = np.array([
            self.current_w1,
            self.current_w2,
            self.current_w3,
            self.current_w4
        ], dtype=np.float64)


    def odom_callback(self, odom_msg):

        self.current_x = odom_msg.data[0]
        self.current_y = odom_msg.data[1]

        self.meas_states[0, :] = odom_msg.data[0]
        self.meas_states[1, :] = odom_msg.data[1]
        self.meas_states[2, :] = self.current_yaw

        self.meas_controls = np.tile(self.feedback_controls.reshape(4, 1), self.N_mhe)

        self.args['p'] = np.concatenate([
            self.meas_states.T.reshape(-1, 1),
            self.meas_controls.T.reshape(-1, 1)
        ])

        self.states = np.tile(self.feedback_states.reshape(3, 1), self.N_mhe+1)
        self.controls = np.tile(self.feedback_controls.reshape(4, 1), self.N_mhe)

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
        
        sol_x = ca.reshape(sol['x'][:3*(self.N_mhe+1)], 3, self.N_mhe+1)
        sol_u = ca.reshape(sol['x'][3*(self.N_mhe+1):], 4, self.N_mhe)

        self.mhe_x = sol_x.full()[0, -1]
        self.mhe_y = sol_x.full()[1, -1]

        print(self.mhe_x, self.mhe_y)

    def mhe_callback(self):
        mhe_msg = Float32MultiArray()

        mhe_msg.data = [float(self.mhe_x), float(self.mhe_y)]

        self.feedback_publisher.publish(mhe_msg)


def main(args=None):

    rclpy.init(args=args)

    node = NMHE_Rabbit()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()

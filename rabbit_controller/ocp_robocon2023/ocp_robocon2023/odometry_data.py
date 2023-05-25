import numpy as np
import rclpy

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion

class Odometry(Node):

    def __init__(self):
        super().__init__('odometry_node')

        self.r = 0.0225
        self.ly = 0.0145
        self.ppr = 8200

        self.old_tick = np.zeros(2)
        self.new_tick = np.zeros(2)
        self.diff = np.zeros(2)
        self.calibrate = True
        
        self.prev_state = np.zeros(3)
        self.curr_state = np.zeros(3)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.rotary_subscriber = self.create_subscription(Float32MultiArray, 'external_rotary', self.rotary_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, 'imu/data2', self.imu_callback, 10)
        
        self.odometry_publisher = self.create_publisher(Float32MultiArray, 'odometry_data', 10)
        # self.rotary_timer = self.create_timer(0.0333, self.rotary_calculation)
        self.odometry_timer = self.create_timer(0.01, self.odometry_callback)

    def rotary_calculation(self, prev_state, u1, u2):

        J = np.array([
            [0,  1],
            [0, -1],
            [0, 1/(2*self.ly)]
        ], dtype=np.float64)

        u1 = (2*np.pi*u1)/self.ppr
        u2 = (2*np.pi*u2)/self.ppr

        u = np.array([u1, u2], dtype=np.float64)

        f = J@u

        new_state = prev_state + f * 0.033

        return new_state


    def rotary_callback(self, tick_msg):

        if (self.calibrate):
            self.new_tick[0] = tick_msg.data[0]
            self.new_tick[1] = tick_msg.data[1]
            self.old_tick = self.new_tick
            self.calibrate = False

        else:
            self.new_tick[0] = tick_msg.data[0]
            self.new_tick[1] = tick_msg.data[1]
            self.diff = self.new_tick - self.old_tick

            for i in range(len(self.new_tick)):
                if (self.diff[i] > 32768):
                    self.diff[i] = self.diff[i] - 65535
                elif (self.diff[i] < -32768):
                    self.diff[i] = self.diff[i] + 65535

            self.curr_state = self.rotary_calculation(self.prev_state, self.diff[0], self.diff[1])
            self.prev_state = self.curr_state


        self.old_tick = self.new_tick

    def imu_callback(self, imu_msg):

        q0 = imu_msg.orientation.x 
        q1 = imu_msg.orientation.y
        q2 = imu_msg.orientation.z
        q3 = imu_msg.orientation.w

        orient_list = [q0, q1, q2, q3]

        roll, pitch, yaw = euler_from_quaternion(orient_list)

        self.yaw = yaw

    def odometryodometry_callback(self):
        odom_msg = Float32MultiArray()

        self.x = self.curr_state[0]
        self.y = self.curr_state[1]

        odom_msg.data = [self.x, self.y, self.yaw]

        self.odometry_publisher.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)

    node = Odometry()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
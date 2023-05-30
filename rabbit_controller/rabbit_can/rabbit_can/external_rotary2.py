import numpy as np
import casadi as ca
import can
import rclpy

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, UInt16MultiArray
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion


class RotaryNode(Node):

    def __init__(self):
        super(RotaryNode, self).__init__('can_node')

        self.r = 0.029
        self.lx = 0.145
        self.ly = 0.145
        self.ppr = 8200

        self.yaw = 0.0

        self.prev_state = ca.DM([0, 0, 0])
        self.curr_state = ca.DM([0, 0, 0])
        self.prev_tick = ca.DM([0, 0])
        self.curr_tick = ca.DM([0, 0])

        self.rotary_data = ca.DM([0, 0])
        self.diff_rotary = ca.DM([0, 0])

        self.first_init = True

        
        self.tick_subscriber = self.create_subscription(UInt16MultiArray, 'external_rotary', self.rotary_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, 'imu/data2', self.imu_callback, 10)

        self.odom_publisher = self.create_publisher(Float32MultiArray, 'odometry_rotary', 10)

        self.rotary_timer = self.create_timer(0.0333, self.rotary_calculation)
        self.odom_timer = self.create_timer(0.02, self.odom_callback)

        self.rotary_model()     


    def rotary_model(self):
        
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        yaw = ca.SX.sym('yaw')

        states = ca.vertcat(x, y, yaw)

        u1 = ca.SX.sym('u1')
        u2 = ca.SX.sym('u2')

        controls = ca.vertcat(u1, u2)

        J = (self.r/2)*ca.DM([
            [1,  0],
            [0, -1],
            [0, 1/(2*self.ly)]
        ])

        rhs = J@controls

        self.f = ca.Function('f', [states, controls], [rhs])


    def rotary_callback(self, tick_msg):
        self.rotary_data[0] = tick_msg.data[0]
        self.rotary_data[1] = tick_msg.data[1]

        if (self.first_init):
            self.prev_tick = np.array([tick_msg.data[0], tick_msg.data[1]], dtype=np.uint16)
            self.curr_tick = self.prev_tick
            self.first_init = False
        else:
            self.curr_tick = np.array([tick_msg.data[0], tick_msg.data[1]], dtype=np.uint16)
            self.diff_rotary = self.curr_tick - self.prev_tick
            for i in range(2):
                if (self.diff_rotary[i] > 32768):
                    self.diff_rotary[i] = self.diff_rotary[i] - 65535
                    print(self.diff_rotary[0] - 65535)
                elif (self.diff_rotary[i] < -32768):
                    self.diff_rotary[i] = self.diff_rotary[i] + 65535

            self.curr_state = self.prev_state + self.f(self.prev_state, self.diff_rotary*2*np.pi/self.ppr)

            self.prev_state = self.curr_state

        self.prev_tick = self.curr_tick

        # print(self.diff_rotary)
        print(self.curr_state)

        # print(self.rotary_model(self.diff_rotary[0], self.diff_rotary[1], 0.0))

    
    def rotary_calculation(self):

        pass


    def imu_callback(self, imu_msg):

        q1 = imu_msg.orientation.x
        q2 = imu_msg.orientation.y
        q3 = imu_msg.orientation.z
        q4 = imu_msg.orientation.w

        orient_list = [q1, q2, q3, q4]

        roll, pitch , yaw = euler_from_quaternion(orient_list)

        self.yaw = yaw

    def odom_callback(self):

        odom_msg = Float32MultiArray()
        odom_msg.data = [0.0, 0.0, 0.0]

        self.odom_publisher.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)

    node = RotaryNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
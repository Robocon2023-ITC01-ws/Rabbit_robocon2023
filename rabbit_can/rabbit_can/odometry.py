import numpy as np
import casadi as ca
import can
import rclpy

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, UInt16MultiArray, String
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion
from model_kinematic.kinematic import *
from nav_msgs.msg import Odometry

class RotaryNode(Node):

    def __init__(self):
        super(RotaryNode, self).__init__('rotary_node')

        self.r = 0.029
        self.ppr = 8200
        self.wheel_ppr = 912

        self.input_cons = [0, 0, 0, 0]

        self.yaw = 0.0

        self.prev_state = np.zeros(2)
        self.curr_state = np.zeros(2)
        self.prev_tick = np.zeros(2)
        self.curr_tick = np.zeros(2)

        self.rotary_data = np.zeros(2)
        self.diff_rotary = np.zeros(2)

        self.first_init = True


        self.prev_wheel = np.zeros(3)
        self.curr_wheel = np.zeros(3)
        self.wheel_vel = np.zeros(3)
        self.prev_wheel_tick = np.zeros(4)
        self.curr_wheel_tick = np.zeros(4)
        self.diff_wheel_tick = np.zeros(4)

        self.prev_yaw = 0.0
        self.curr_yaw = 0.0
        self.dyaw = 0.0
        self.hist_yaw = [0]

        self.wheel_init = True
        self.retry_status = None

        self.kinematic = kinematic()

        self.wheel_subscriber = self.create_subscription(UInt16MultiArray, 'wheel_tick', self.wheel_callback, 10)
        self.tick_subscriber = self.create_subscription(UInt16MultiArray, 'external_rotary', self.rotary_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, 'imu/data2', self.imu_callback, 10)
        self.state_subscriber = self.create_subscription(String, 'retry_state', self.retry_callback, 10)

        self.odom_publisher = self.create_publisher(Float32MultiArray, 'odometry_rotary', 10)
        self.wheel_odom = self.create_publisher(Float32MultiArray, 'odom_wheel', 10)
        self.input_publisher = self.create_publisher(Float32MultiArray, 'feedback_controls', 10)
        self.input_timer = self.create_timer(0.05, self.input_callback)

        # self.rotary_timer = self.create_timer(0.0333, self.rotary_cb_pub)
        # self.odom_timer = self.create_timer(0.02, self.odom_callback)

    def wheel_model(self, u1, u2, u3, u4, theta):

        w1 = u1 * 2 * np.pi/self.wheel_ppr
        w2 = u2 * 2 * np.pi/self.wheel_ppr
        w3 = u3 * 2 * np.pi/self.wheel_ppr
        w4 = u4 * 2 * np.pi/self.wheel_ppr

        self.input_cons = [w1, w2, w3, w4]

        for_vec = self.kinematic.meca_forward_kinematic(w1, w2, w3, w4, theta)

        return for_vec
    
    def input_callback(self):
        input_msg = Float32MultiArray()

        input_msg.data = [float(self.input_cons[0]), float(self.input_cons[1]), float(self.input_cons[2]), float(self.input_cons[3])]

        self.input_publisher.publish(input_msg)

    def retry_callback(self, re_msg):
        self.retry_status = re_msg.data
    
    def imu_callback(self, imu_msg):

        q1 = imu_msg.orientation.x
        q2 = imu_msg.orientation.y
        q3 = imu_msg.orientation.z
        q4 = imu_msg.orientation.w

        orient_list = [q1, q2, q3, q4]

        roll, pitch , yaw = euler_from_quaternion(orient_list)

        self.yaw = yaw
        # self.curr_yaw = yaw

        # self.dyaw = self.curr_yaw - self.prev_yaw

        # self.prev_yaw = self.curr_yaw

        self.hist_yaw.append(yaw)

        self.dyaw = self.hist_yaw[-1]-self.hist_yaw[-2]

    def wheel_callback(self, tick_msg):
        odom_msg = Float32MultiArray()
        if (self.wheel_init):
            self.prev_wheel_tick = np.array([tick_msg.data[0], tick_msg.data[1], tick_msg.data[2], tick_msg.data[3]])
            self.curr_wheel_tick = self.prev_wheel_tick
            self.wheel_init = False
        else:
            self.curr_wheel_tick = np.array([tick_msg.data[0], tick_msg.data[1], tick_msg.data[2], tick_msg.data[3]])
            self.diff_wheel_tick = (self.curr_wheel_tick - self.prev_wheel_tick)
            for i in range(4):
                if (self.diff_wheel_tick[i] > 32768):
                    self.diff_wheel_tick[i] = self.diff_wheel_tick[i] - 65535
                elif (self.diff_wheel_tick[i] < -32768):
                    self.diff_wheel_tick[i] = self.diff_wheel_tick[i] + 65535
                
            self.curr_wheel = self.prev_wheel - self.wheel_model(self.diff_wheel_tick[0], self.diff_wheel_tick[1],
                                                self.diff_wheel_tick[2], self.diff_wheel_tick[3], self.yaw)
            
            odom_msg.data = [self.curr_wheel[0], self.curr_wheel[1]]

            self.wheel_odom.publish(odom_msg)

            self.prev_wheel = self.curr_wheel
        self.prev_wheel_tick = self.curr_wheel_tick
        print(self.curr_wheel)
        # print(self.wheel_vel)

        self.prev_wheel_tick = self.curr_wheel_tick
 

    def rotary_model(self, t1, t2, theta):

        u1 = t1 * 2 * np.pi / self.ppr
        u2 = t2 * 2 * np.pi / self.ppr

        for_vec = (self.r)*np.array([
            u1*np.cos(theta)-u2*np.sin(theta),
            u1*np.sin(theta)+u2*np.cos(theta),
        ], dtype=np.float64)

        return for_vec


    def rotary_callback(self, tick_msg):
        odom_msg = Float32MultiArray()

        if (self.first_init):
            self.prev_tick = np.array([tick_msg.data[0], tick_msg.data[1]])
            self.curr_tick = self.prev_tick
            self.first_init = False
        else:
            self.curr_tick = np.array([tick_msg.data[0], tick_msg.data[1]])
            self.diff_rotary = self.curr_tick - self.prev_tick
            for i in range(2):
                if (self.diff_rotary[i] > 32768):
                    self.diff_rotary[i] = self.diff_rotary[i] - 65535
                elif (self.diff_rotary[i] < -32768):
                    self.diff_rotary[i] = self.diff_rotary[i] + 65535
            #self.curr_state[0] = self.prev_state[0]+ self.rotary_model(self.diff_rotary[0], self.diff_rotary[1], self.yaw)[0]
            #self.curr_state[1] = self.prev_state[1]+ self.rotary_model(self.diff_rotary[0], self.diff_rotary[1], self.yaw)[1]
            self.curr_state = self.prev_state+ self.rotary_model(self.diff_rotary[0], self.diff_rotary[1], self.yaw)
            odom_msg.data = [float(self.curr_state[0]), float(self.curr_state[1])]

            self.odom_publisher.publish(odom_msg)

            self.prev_state = self.curr_state

        self.prev_tick = self.curr_tick

        #print(self.curr_state)

        # print(self.rotary_model(self.diff_rotary[0], self.diff_rotary[1], 0.0))



def main(args=None):
    rclpy.init(args=args)

    node = RotaryNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()

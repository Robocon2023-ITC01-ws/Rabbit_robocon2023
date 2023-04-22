import rclpy
from rclpy.node import Node
import threading
from model.rabbit_model import *
import os

# ROS2 message
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Vector3, Twist
from sensor_msgs.msg import Imu

class ros_node(Node):
    def __init__(self):
        super(ros_node, self).__init__('yaw_node')
        self.rabbit = RabbitModel()
        self.q = np.zeros(4)
        self.yaw = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.store_data = 0.0
        self.i = 0;

        # ROS2  publisher and Subscriber
        self.imu_subsriber = self.create_subscription(Imu, "/imu/data2", self.imu_callback, 100)
        

    def imu_callback(self, imu_msg):
        self.q[0] = imu_msg.orientation.x
        self.q[1] = imu_msg.orientation.y
        self.q[2] = imu_msg.orientation.z
        self.q[3] = imu_msg.orientation.w
        self.roll, self.pitch, self.yaw = self.rabbit.euler_from_quaternion(self.q[0], self.q[1], self.q[2], self.q[3])
        self.yaw = -self.yaw
        while(self.i <= 6):
            self.store_data = self.yaw
            self.i = self.i + 1
            break
        self.yaw = self.yaw - self.store_data
        print(self.yaw)
        


def main(args=None):
    rclpy.init(args=args)
    yaw_node = ros_node()
    rclpy.spin(yaw_node)
    yaw_node.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()
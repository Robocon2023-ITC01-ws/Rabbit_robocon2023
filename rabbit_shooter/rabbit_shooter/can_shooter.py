import rclpy
import numpy as np
from rclpy.node import Node

from std_msgs.msg import UInt16 # laser: subscribe the data from laser
# from std_msgs.msg import Int32 # motor: publish the data to the motor 
from std_msgs.msg import Int8 # button: to control shoot or not shoot
from std_msgs.msg import Int32MultiArray, Int32
from time import sleep

from shooter.RR_shooter import *

class ShooterNode(Node):
    def __init__(self):
        super().__init__('shooter_node')
        self.laser_sub = self.create_subscription(UInt16, 'laser', self.laser_callback, 10)
        self.button_command = 0
        self.laser_data = 0
        self.shooter_data = 0
        self.distance = 0.0
        self.direction = 0

    def laser_callback(self, laser_msg):
        self.laser_data = laser_msg.data
        print(self.laser_data)
        distance = (3.516 - 0.214)/(2194 - 10)*(self.laser_data - 10) + 0.214
        print(distance)
        

def main(args=None):
    rclpy.init(args=args)
    shooter_node = ShooterNode()
    rclpy.spin(shooter_node)
    shooter_node.destroy_node()
    rclpy.shutdown

if __name__=='__main__':
    main()
import rclpy
import numpy as np
from rclpy.node import Node

from std_msgs.msg import UInt16 # laser: subscribe the data from laser
from std_msgs.msg import UInt32 # motor: publish the data to the motor 
from std_msgs.msg import Int8 # button: to control shoot or not shoot
from std_msgs.msg import Int32MultiArray, Int32, Float32
from time import sleep

from shooter.RR_shooter import *

class ShooterNode(Node):
    def __init__(self):
        super().__init__('shooter_node')

        self.laser_sub = self.create_subscription(UInt16, 'laser', self.laser_callback, 10)
        
        self.button_sub = self.create_subscription(Int8, "shooter_command", self.button_callback, 10)
        self.shooter_pub = self.create_publisher(UInt32, 'shooter', 10)

        self.button_command = 0
        self.laser_data = 0
        self.shooter_data = 0
        self.distance = 0.0
        self.direction = 0

    def button_callback(self, button_msg):
        button_command = int(button_msg.data)
        while(button_command == 1):
            self.laser_sub = self.create_subscription(UInt16, 'laser', self.laser_callback, 10)
            distance = ((5.499 - 0.241)/(3516 - 9)*(self.laser_data - 9) + 0.241
            print(distance)
            self.rps = shooter(distance).shooter()
            print(self.rps)
            if(self.rps == 7433):
                self.rps = 0 
            shooter_msg = UInt32()
            shooter_msg.data = int(self.rps)
            self.shooter_pub.publish(shooter_msg)
            self.rps = 0
            break

        

    def laser_callback(self, laser_msg):
        self.laser_data = laser_msg.data
        

def main(args=None):
    rclpy.init(args=args)
    shooter_node = ShooterNode()
    rclpy.spin(shooter_node)
    shooter_node.destroy_node()
    rclpy.shutdown

if __name__=='__main__':
    main()


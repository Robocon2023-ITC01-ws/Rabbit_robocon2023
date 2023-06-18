import rclpy
import numpy as np
import getch
from rclpy.node import Node

from std_msgs.msg import Int8




class ShooterCommandNode(Node):
    def __init__(self):
        super().__init__('command_node')
        self.button_pub = self.create_publisher(Int8, 'shooter_command', 10)
        self.pick_up_pub = self.create_publisher(Int8, 'pick_up', 10)

        self.timer = self.create_timer(0.01, self.button_callback)
        
    def button_callback(self):
        msg = Int8()
        pick_msg = Int8()
        char = getch.getch()
        if(char == 'a'):
            msg.data = 1
            self.button_pub.publish(msg)
        elif(char == '1'):
            pick_msg.data = 1
            self.pick_up_pub.publish(pick_msg)
            msg.data = 0
            self.button_pub.publish(msg)
        elif(char == '0'):
            pick_msg.data = 0
            self.pick_up_pub.publish(pick_msg)
            msg.data = 0
            self.button_pub.publish(msg)
        elif(char == '2'):
            pick_msg.data = 2
            self.pick_up_pub.publish(pick_msg)
            msg.data = 0
            self.button_pub.publish(msg)
        elif(char == '3'):
            pick_msg.data = 3
            self.pick_up_pub.publish(pick_msg)
            msg.data = 0
            self.button_pub.publish(msg)
        else:
            msg.data = 0
            self.button_pub.publish(msg)
            pick_msg.data = 0
            self.pick_up_pub.publish(pick_msg)

        

def main(args=None):
    rclpy.init(args=args)
    shooter_node = ShooterCommandNode()
    rclpy.spin(shooter_node)
    shooter_node.destroy_node()
    rclpy.shutdown
    

if __name__=='__main__':
    main()


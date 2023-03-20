import rclpy
import can
from rclpy.node import Node

import math
import numpy as np
from std_msgs.msg import Int32


def map(Input, Min_Input, Max_Input, Min_Output, Max_Output):
    value =  ((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output)
    return value

class ros_node(Node):
    def __init__(self):
        super(ros_node, self).__init__('shooter_node')
        self.TxData = [0, 0]
        self.V_in = 0
        self.V_out = 0
        self.bus = can.Bus(channel='can0',interface='socketcan',bitrate=1000000)
        self.shooter_pub = self.create_subscription(Int32, 'shooter', self.shooter_callback, 100)

    def shooter_callback(self, shooter_msg):
        self.V_in = shooter_msg.data
        self.V_out = int(map(self.V_in, 0, 1500, 0, 65535))
        self.TxData[0] = ((self.V_out & 0xFF00) >> 8)
        self.TxData[1] = (self.V_out & 0x00FF)
        msg = can.Message(arbitration_id=0x222,data=self.TxData,dlc=2,is_extended_id=False)
        self.bus.send(msg,0.01)

def main(args=None):
    rclpy.init(args=args)
    shooter_node = ros_node()
    rclpy.spin(shooter_node)
    shooter_node.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()

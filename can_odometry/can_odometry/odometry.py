import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from

import can
import numpy as np

wheel_d = 0.058  # in meter
d_wheel = 0.285 # in meter
def position( tick, dir, count, pos):
    new_count = tick
    if (dir == False):
        if(new_count <= count):
            diff = count - new_count
        else:
            diff = (65536 - new_count) + count
        pos = np.pi * wheel_d * diff/8200 * (1.0) + pos

    else:
        if(new_count>=count):
            diff = new_count - count
        else:
            diff = (65536 - count) + new_count 
        pos = np.pi * wheel_d * diff/8200 * (-1.0) + pos
    count = new_count
    return pos, count

class ros_node(Node):
    def __init__(self):
        super(ros_node, self).__init__('odom_node')
        self.timer = 0.01
        self.bus = can.interface.Bus(channel='can0', interface = 'socketcan', bitrate = 1000000)

        self.encoder_tick = np.zeros(3)
        self.encoder_dir = np.zeros(3)
        self.position = np.zeros(3)
        self.count = np.zeros(3)
        self.Xr = 0.0 # in meter
        self.Y = 0.0 # in meter
        self.Xl = 0.0 # in meter
        self.theta = 0.0 # in meter
        self.store = np.zeros(3)
        i = 0
        

        self.odom_pub = self.create_publisher(Float32MultiArray, 'odom', 10)
        #self.odom_timer = self.create_timer(self.timer, self.odom_callback)
        while(rclpy.ok()):
            msg = self.bus.recv(0.01)
            self.encoder_tick[0] = (msg.data[0] << 8) + msg.data[1]
            self.encoder_tick[1] = (msg.data[2] << 8) + msg.data[3]
            self.encoder_tick[2] = (msg.data[4] << 8) + msg.data[5]
            self.encoder_dir[0] = (msg.data[6] & 0x04) == 0x04
            self.encoder_dir[1] = (msg.data[6] & 0x02) == 0x02
            self.encoder_dir[2] = (msg.data[6] & 0x01) == 0x01
            while(i <= 6):
                self.store[0] = self.encoder_tick[0]
                self.store[1] = self.encoder_tick[1]
                self.store[2] = self.encoder_tick[2]
                i = i +1
                break;
            self.encoder_tick[0] = self.encoder_tick[0] - self.store[0]
            self.encoder_tick[1] = self.encoder_tick[1] - self.store[1]
            self.encoder_tick[2] = self.encoder_tick[2] - self.store[2]
            self.Xr, self.count[0] = position(self.encoder_tick[0], self.encoder_dir[0], self.count[0], self.Xr)
            self.Xl, self.count[1] = position(self.encoder_tick[1], self.encoder_dir[1], self.count[1], self.Xl)
            self.Y, self.count[2] = position(self.encoder_tick[2], self.encoder_dir[2], self.count[2], self.Y)
            xl = -self.Xl
            self.theta = (self.Xr - xl)/(2*d_wheel)*57.298
            pos_msg = Float32MultiArray()
            pos_msg.data = [self.Xr, -self.Y, self.theta]

            self.position = np.array([np.round(self.Xr,3), np.round(-self.Y,3), np.round(self.theta,3), xl])
            print(self.position)
             
            
            self.odom_pub.publish(pos_msg)
        
    
        


def main(args=None):
    rclpy.init(args=args)
    odom_node = ros_node()
    rclpy.spin(odom_node)
    odom_node.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()


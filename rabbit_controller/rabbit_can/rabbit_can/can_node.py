import rclpy
from rclpy.node import Node
import can
from geometry_msgs.msg import Twist, Vector3
import numpy as np
from model_kinematic.kinematic import *
from std_msgs.msg import Float32MultiArray, UInt16MultiArray, Int8, Int16, Int32

class ros_node(Node):
    def __init__(self):
        super(ros_node, self).__init__('can_node')
        self.timer = 0.001
        self.vel_timer = 0.01
        self.bus = can.interface.Bus(channel='can0', interface = 'socketcan', bitrate = 1000000)
        self.command_sub = self.create_subscription(Twist, 'cmd_vel', self.command_callback, 100)
        self.can_timer = self.create_timer(self.timer, self.can_callback)
        self.rotary_publisher = self.create_publisher(UInt16MultiArray, 'external_rotary', 10)
        self.tick_publisher = self.create_publisher(UInt16MultiArray, 'wheel_tick', 10)
        self.tick_timer = self.create_timer(0.01, self.tick_callback)
        self.rotary_timer = self.create_timer(0.02, self.rotary_model)
        self.TxData = [128, 0, 128, 0, 128, 0, 128, 0]
        self.maxV = 1
        self.maxOmega = np.pi
        self.kinematic = kinematic()

        # Subscriber 
        self.input = self.create_subscription(Float32MultiArray, 'input_controls', self.input_callback, 10)

        # ==== Back data ====
        self.wheel_vel = np.zeros(4)
        self.wheel_cal = np.zeros(4)
        self.velocity_callback = np.zeros(4)
        self.command_vel = np.zeros(3)
        self.input_vel = np.zeros(4)

        self.tick_data = [0, 0, 0, 0]

        # Rotary data
        self.rotary_data = [0, 0]

        # laser to shooter
        self.laser_pub = self.create_publisher(Int16, 'laser', 10)
        self.shooter_sub = self.create_subscription(Int32, 'shooter', self.shooter_callback, 10)
        self.TxData1 = [0, 0, 0, 0]

        # pick up (int8 from controller digital control 1 to up and 0 to down)
        self.pick_sub = self.create_subscription(Int8, 'pick_up', self.pick_up_callback, 10)

        #============ Shooter part ==============
        self.pub_shooter_speed = 0
        self.shooter_data = 0
        self.pick_up_command = 0


    def input_callback(self, input_msg):
        
        # self.input_vel = input_msg.data

        V1 = int(self.kinematic.map(input_msg.data[0], -100, 100, 0, 65535))
        V2 = int(self.kinematic.map(input_msg.data[1], -100, 100, 0, 65535))
        V3 = int(self.kinematic.map(input_msg.data[2], -100, 100, 0, 65535))
        V4 = int(self.kinematic.map(input_msg.data[3], -100, 100, 0, 65535))

        self.TxData[0] = ((V1 & 0xFF00) >> 8)
        self.TxData[1] = (V1 & 0x00FF)
        self.TxData[2] = ((V4 & 0xFF00) >> 8)
        self.TxData[3] = (V4 & 0x00FF)
        self.TxData[4] = ((V3 & 0xFF00) >> 8)
        self.TxData[5] = (V3 & 0x00FF)
        self.TxData[6] = ((V2 & 0xFF00) >> 8)
        self.TxData[7] = (V2 & 0x00FF)


    def command_callback(self, twist_msg):
        vx = twist_msg.linear.x
        vy = twist_msg.linear.y
        vyaw = twist_msg.angular.z

        # vx = self.kinematic.map(vx, -1, 1, -1.0*self.maxV, self.maxV)
        # vy = self.kinematic.map(vy, -1, 1, -1.0*self.maxV, self.maxV)
        # vyaw = self.kinematic.map(vyaw, -1, 1, -1.0*self.maxOmega, self.maxOmega)

        # print(vx, vy, omega)

        v1, v2, v3, v4 = self.kinematic.omni_inverse_kinematic(vx, vy, vyaw, 0.0)

        # print(v1, v2, v3, v4)

        V1 = int(self.kinematic.map(v1, -100, 100, 0, 65535))
        V2 = int(self.kinematic.map(v2, -100, 100, 0, 65535))
        V3 = int(self.kinematic.map(v3, -100, 100, 0, 65535))
        V4 = int(self.kinematic.map(v4, -100, 100, 0, 65535))

        self.TxData[0] = ((V1 & 0xFF00) >> 8)
        self.TxData[1] = (V1 & 0x00FF)
        self.TxData[2] = ((V4 & 0xFF00) >> 8)
        self.TxData[3] = (V4 & 0x00FF)
        self.TxData[4] = ((V3 & 0xFF00) >> 8)
        self.TxData[5] = (V3 & 0x00FF)
        self.TxData[6] = ((V2 & 0xFF00) >> 8)
        self.TxData[7] = (V2 & 0x00FF)

    def pick_up_callback(self, pick_up_msg):
        self.pick_up_command = pick_up_msg.data

    def shooter_callback(self, shooter_msg):
        if (shooter_msg.data > 10):
            self.TxData1[2] = 1
     
            shooter_speed = shooter_msg.data
        elif (shooter_msg.data < -10):
            self.TxData1[2] = 0
            
            shooter_speed = -1*shooter_msg.data
        else :
            self.TxData1[2] = 0
            shooter_speed = 0
        
        self.TxData1[3] = self.pick_up_command
        print(self.shooter_data)
        self.TxData1[0] = ((shooter_speed & 0xFF00) >> 8)
        self.TxData1[1] = (shooter_speed & 0x00FF)
        
        self.shoot_msg = can.Message(arbitration_id=0x222, data= self.TxData1, dlc= 5, is_extended_id= False)
        self.pub_shooter_speed = 1

    def can_callback(self):
        msg = can.Message(arbitration_id=0x111, is_extended_id=False, data=self.TxData)
        laser_msg = Int16()
        try:
            if (self.pub_shooter_speed):
                self.pub_shooter_speed = 0
                self.bus.send(self.shoot_msg,0.01)
            self.bus.send(msg, 0.01)
            finish_recv = True

        except can.CanError:
            pass

        for i in range(3):
            try:
                can_msg = self.bus.recv(0.1)
                if (can_msg != None):
                    if can_msg.arbitration_id == 0x155:
                        self.tick_data[0] = (can_msg.data[0] << 8) + can_msg.data[1]
                        self.tick_data[3] = (can_msg.data[2] << 8) + can_msg.data[3]

                    elif can_msg.arbitration_id == 0x140:
                        self.tick_data[2] = (can_msg.data[0] << 8) + can_msg.data[1]
                        self.tick_data[1] = (can_msg.data[2] << 8) + can_msg.data[3]

                    elif can_msg.arbitration_id == 0x333:
                        
                        self.rotary_data[0] = can_msg.data[2] << 8 | can_msg.data[3]
                        self.rotary_data[1] = can_msg.data[0] << 8 | can_msg.data[1]
                        laser_int = can_msg.data[6] << 8 | can_msg.data[7]

                        # laser to shooter
                        laser_msg.data = laser_int
                        self.laser_pub.publish(laser_msg)

                else:
                    self.get_logger().error('time out on msg recv!')
            except can.CanOperationError:
                pass
            print(self.tick_data)
        for i in range(len(self.velocity_callback)):
            if(self.velocity_callback[i] <= 0.00153 and self.velocity_callback[i] >= -0.00153):
                self.velocity_callback[i] = 0.0
    
    def tick_callback(self):
        tick_msg = UInt16MultiArray()

        tick_msg.data = [self.tick_data[0], self.tick_data[1],self.tick_data[2], self.tick_data[3]]

        self.tick_publisher.publish(tick_msg)

    def rotary_model(self):

        odom_msg = UInt16MultiArray()

        odom_msg.data = [self.rotary_data[0], self.rotary_data[1]]

        self.rotary_publisher.publish(odom_msg)



def main(args=None):
    rclpy.init(args=args)
    can_node = ros_node()
    rclpy.spin(can_node)
    can_node.destroy_node()
    rclpy.shutdown

if __name__=='__main__':
    main()

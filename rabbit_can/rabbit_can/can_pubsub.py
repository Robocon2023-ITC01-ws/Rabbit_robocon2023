import rclpy
from rclpy.node import Node
import can
from geometry_msgs.msg import Twist
import numpy as np
from model_kinematic.kinematic import *
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16, Int32MultiArray
class ros_node(Node):
    def __init__(self):
        super(ros_node, self).__init__('can_node')
        self.timer = 0.001
        self.vel_timer = 0.01
        self.bus = can.interface.Bus(channel='can0', interface = 'socketcan', bitrate = 1000000)
        self.command_sub = self.create_subscription(Twist, 'cmd_vel', self.command_callback, 100)
        self.can_timer = self.create_timer(self.timer, self.can_callback)
        self.control_pub = self.create_publisher(Float32MultiArray, 'input_control1',1)
        self.vel_pub = self.create_publisher(Twist, 'vel_data', 1)
        self.timer_control = self.create_timer(self.vel_timer, self.control_callback)
        self.timer_vel = self.create_timer(self.vel_timer, self.velocity)
        self.TxData = [128, 0, 128, 0, 128, 0, 128, 0]
        self.maxV = 1
        self.maxOmega = np.pi
        self.kinematic = kinematic()

        # laser to shooter
        self.laser_pub = self.create_publisher(Int16, 'laser', 10)
        self.shooter_sub = self.create_subscription(Int32MultiArray, 'shooter', self.shooter_callback, 10)
        self.TxData1 = [0, 0, 0]


        # Subscriber 
        self.input = self.create_subscription(Float32MultiArray, 'input_control', self.input_callback, 10)

        # ==== Back data ====
        self.wheel_vel = np.zeros(4)
        self.wheel_cal = np.zeros(4)
        self.velocity_callback = np.zeros(4)
        self.command_vel = np.zeros(3)
        self.input_vel = np.zeros(4)
        self.pub_shooter_speed = 0
        self.shooter_data = np.array([0,0])
    
    def input_callback(self, input_msg):
        self.input_vel = input_msg.data

        # V1 = int(self.kinematic.map(self.input_vel[0], -100, 100, 0, 65535))
        # V2 = int(self.kinematic.map(self.input_vel[1], -100, 100, 0, 65535))
        # V3 = int(self.kinematic.map(self.input_vel[2], -100, 100, 0, 65535))
        # V4 = int(self.kinematic.map(self.input_vel[3], -100, 100, 0, 65535))
        # self.TxData[0] = ((V1 & 0xFF00) >> 8)
        # self.TxData[1] = (V1 & 0x00FF)
        # self.TxData[2] = ((V2 & 0xFF00) >> 8)
        # self.TxData[3] = (V2 & 0x00FF)
        # self.TxData[4] = ((V3 & 0xFF00) >> 8)
        # self.TxData[5] = (V3 & 0x00FF)
        # self.TxData[6] = ((V4 & 0xFF00) >> 8)
        # self.TxData[7] = (V4 & 0x00FF)

    def shooter_callback(self, shooter_msg):
        self.shooter_data = shooter_msg.data
        shooter_speed = self.shooter_data[0]
        self.TxData1[2] = self.shooter_data[1]
        print(self.shooter_data)
        self.TxData1[0] = ((shooter_speed & 0xFF00) >> 8)
        self.TxData1[1] = (shooter_speed & 0x00FF)
        self.shoot_msg = can.Message(arbitration_id=0x222, data= self.TxData1, dlc= 3, is_extended_id= False)
        self.pub_shooter_speed = 1


    def command_callback(self, twist_msg):
        x = twist_msg.linear.x
        y = twist_msg.linear.y
        vel_yaw = twist_msg.angular.z

        vx = self.kinematic.map(x, -1, 1, -1.0*self.maxV, self.maxV)
        vy = self.kinematic.map(y, -1, 1, -1.0*self.maxV, self.maxV)
        omega = self.kinematic.map(vel_yaw, -1, 1, -1.0*self.maxOmega, self.maxOmega)

        v1, v2, v3, v4 = self.kinematic.mecanum_inverse_kinematic(vx, vy, omega)
        

        V1 = int(self.kinematic.map(v1, -100, 100, 0, 65535))
        V2 = int(self.kinematic.map(v2, -100, 100, 0, 65535))
        V3 = int(self.kinematic.map(-v3, -100, 100, 0, 65535))
        V4 = int(self.kinematic.map(v4, -100, 100, 0, 65535))

        self.TxData[0] = ((V1 & 0xFF00) >> 8)
        self.TxData[1] = (V1 & 0x00FF)
        self.TxData[2] = ((V2 & 0xFF00) >> 8)
        self.TxData[3] = (V2 & 0x00FF)
        self.TxData[4] = ((V3 & 0xFF00) >> 8)
        self.TxData[5] = (V3 & 0x00FF)
        self.TxData[6] = ((V4 & 0xFF00) >> 8)
        self.TxData[7] = (V4 & 0x00FF)

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
        while(finish_recv):
            try:
                can_msg = self.bus.recv(0.1)
                if (can_msg != None):
                    if can_msg.arbitration_id == 0x155:
                        self.wheel_vel[0] = (can_msg.data[0] << 8) + can_msg.data[1]
                        self.wheel_vel[1] = (can_msg.data[2] << 8) + can_msg.data[3]
                        self.wheel_cal[0] = float(self.kinematic.map(self.wheel_vel[0], 0, 65535, -100, 100))
                        self.wheel_cal[1] = float(self.kinematic.map(self.wheel_vel[1], 0, 65535, -100, 100))

                    elif can_msg.arbitration_id == 0x140:
                        self.wheel_vel[2] = (can_msg.data[0] << 8) + can_msg.data[1]
                        self.wheel_vel[3] = (can_msg.data[2] << 8) + can_msg.data[3]
                        self.wheel_cal[2] = float(self.kinematic.map(self.wheel_vel[2], 0, 65535, -100, 100))
                        self.wheel_cal[3] = float(self.kinematic.map(self.wheel_vel[3], 0, 65535, -100, 100))
                        
                    self.velocity_callback = self.wheel_cal

                else:
                    self.get_logger().error('time out on msg recv!')
                    finish_recv = False
                    
            except can.CanOperationError:
                pass
            #print(self.velocity_callback)
        for i in range(len(self.velocity_callback)):
            if(self.velocity_callback[i] <= 0.00153 and self.velocity_callback[i] >= -0.00153):
                self.velocity_callback[i] = 0.0
        self.command_vel[0], self.command_vel[1], self.command_vel[2] = self.kinematic.mecanum_forward_kinematic(self.velocity_callback[0], self.velocity_callback[1], self.velocity_callback[2], self.velocity_callback[3])

    def velocity(self):
        vel_msg = Twist()
        vel_msg.linear.x = self.command_vel[0]
        vel_msg.linear.y = self.command_vel[1]
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = self.command_vel[2]
        self.vel_pub.publish(vel_msg)
    
    def control_callback(self):
        con_msg = Float32MultiArray()
        con_msg.data = [self.velocity_callback[0],self.velocity_callback[1],self.velocity_callback[2],self.velocity_callback[3]]
        self.control_pub.publish(con_msg)


def main(args=None):
    rclpy.init(args=args)
    can_node = ros_node()
    rclpy.spin(can_node)
    can_node.destroy_node()
    rclpy.shutdown

if __name__=='__main__':
    main()



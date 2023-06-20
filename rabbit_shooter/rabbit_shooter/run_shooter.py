import rclpy
import numpy as np
from rclpy.node import Node
import yaml
## This node is created to publish the data to store in the yaml file in order to fixed the parameter for the future use
## In which the parameter will be come from the testing file and used the data from yaml file to use in the automatic shooter
from std_msgs.msg import UInt16 # laser: subscribe the data from laser
from std_msgs.msg import UInt8 # motor: publish the data to the motor 
from std_msgs.msg import Int8 # button: to control shoot or not shoot
from std_msgs.msg import Float32 # Parameter publisher
from std_msgs.msg import UInt32

max = 10
min = 3
gain = 0.5

import getpass
username = getpass.getuser()


def read_one_block_of_yaml_data(filename, key):
    with open(f'{filename}','r') as f:
        output = yaml.safe_load(f)
    return output[f'{key}'] 
    

class ShooterNode(Node):
    def __init__(self):
        super().__init__('shooter_node')

        self.laser_sub = self.create_subscription(UInt16, 'laser', self.laser_callback, 10)
        self.button_sub = self.create_subscription(Int8, "shooter_command", self.button_callback, 10)
        self.adjust_left_sub = self.create_subscription(Float32, "adjust_left", self.adjust_left_callback,10)
        self.adjust_right_sub = self.create_subscription(Float32, "adjust_right", self.adjust_right_callback,10)
        self.shooter_pub = self.create_publisher(UInt32, 'shooter', 10)
        # self.save_sub = self.create_subscription(Int8, "data_save", self.save_callback, 10)
        self.file = f'/home/{username}/rabbit_ws/src/rabbit_shooter/config/data.yaml' #1
        self.file1 = f'/home/{username}/rabbit_ws/src/rabbit_shooter/config/data1.yaml' # 1_5
        self.file2= f'/home/{username}/rabbit_ws/src/rabbit_shooter/config/data2.yaml'#2

        self.button_command = 0
        self.laser_data = 0
        self.shooter_data = 0
        self.distance = 0.0
        self.adjust = 0.0
        self.adjust_left = 0.0
        self.adjust_right = 0.0
        self.xin = 0.0
        self.count = 0
        self.save = 0
        self.speed = 0
        self.push_save = 0
        self.saved = 0
        self.get_param()
        

    def get_param(self):
        pole1 = read_one_block_of_yaml_data(self.file, key='len_i')
        pole1_5 = read_one_block_of_yaml_data(self.file1, key='len_i')
        pole2 = read_one_block_of_yaml_data(self.file2, key='len_i')

        pole1_distance = np.zeros(pole1)
        pole1_speed = np.zeros(pole1)

        pole2_distance = np.zeros(pole2)
        pole2_speed = np.zeros(pole2)

        pole1_5_distance = np.zeros(pole1_5)
        pole1_5_speed = np.zeros(pole1_5)
        print(pole1)
        print(pole1_5)
        print(pole2)

        for i in range(pole1):
            data = read_one_block_of_yaml_data(self.file, key=f'point_{i}')
            pole1_distance[i] = data[0]
            pole1_speed[i] = data[1]

        z = np.polyfit(pole1_distance, pole1_speed, 1)
        self.pole1_shoot = np.poly1d(z)

        for i in range(pole1_5):
            data = read_one_block_of_yaml_data(self.file1, key=f'point_{i}')
            print(data)
            pole1_5_distance[i] = data[0]
            pole1_5_speed[i] = data[1]
        
        z1 = np.polyfit(pole1_5_distance, pole1_5_speed, 1)
        self.pole1_5_shoot = np.poly1d(z1)

        for i in range(pole2):
            data = read_one_block_of_yaml_data(self.file2, key=f'point_{i}')
            pole2_distance[i] = data[0]
            pole2_speed[i] = data[1]
        print(pole2_distance, pole2_speed)
        z2 = np.polyfit(pole2_distance, pole2_speed, 1)
        self.pole2_shoot = np.poly1d(z2)

    
    
    def adjust_left_callback(self, adjust_msg):
        self.adjust_left = adjust_msg.data

    def adjust_right_callback(self, adjust_msg):
        self.adjust_right = adjust_msg.data

   

    def map(self, Input, Min_Input, Max_Input, Min_Output, Max_Output):
        value =  ((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output)
        return value

    def button_callback(self, button_msg):
        button_command = int(button_msg.data)
        if(button_command == 1 ):
            self.laser_sub = self.create_subscription(UInt16, 'laser', self.laser_callback, 10)
            self.distance = (3.516 - 0.214)/(2194 - 10)*(self.laser_data - 10) + 0.214
            left = self.map(self.adjust_left, 1, -1, 0 ,0.6)
            right = self.map(self.adjust_right, 1, -1, 0, 0.6)
            self.distance = self.distance + (right - left)
            if (self.adjust <0):
                self.adjust = 0
            self.speed = self.speed_shooter(self.distance)
            self.rps = self.map(self.speed, 0, 1500, 0, 65535)
            
            shooter_msg = UInt32()
            shooter_msg.data = int(self.rps)
            self.shooter_pub.publish(shooter_msg)
        

    def speed_shooter(self, x):
        if(x >= 0.2 and x<=0.8): # Pole 1meter
            y = self.pole1_shoot(x)
        elif(x >= 1.4 and x<= 2.2 ): # Pole 1.5 meter
            y = self.pole1_5_shoot(x)
        elif(x > 0.8 and x<1.4): # Pole 1meter on the next side
            y = self.pole2_shoot(x)
        return y

        

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
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


def write_yaml_to_file(py_obj,filename):
    with open(f'{filename}', 'w',) as f :
        yaml.dump(py_obj,f,sort_keys=True) 
    print('Written to file successfully')

def read_and_modify_one_block_of_yaml_data(filename, key, value):
    with open(f'{filename}', 'r') as f:
        data = yaml.safe_load(f)
        data[f'{key}'] = value 
        print(data) 
    with open(f'{filename}', 'w') as file:
        yaml.dump(data,file,sort_keys=False)
    print('done!')
    print('done!')
    

class ShooterNode(Node):
    def __init__(self):
        super().__init__('shooter_node')

        self.laser_sub = self.create_subscription(UInt16, 'laser', self.laser_callback, 10)
        self.button_sub = self.create_subscription(Int8, "shooter_command", self.button_callback, 10)
        self.adjust_left_sub = self.create_subscription(Float32, "adjust_left", self.adjust_left_callback,10)
        self.adjust_right_sub = self.create_subscription(Float32, "adjust_right", self.adjust_right_callback,10)
        self.shooter_pub = self.create_publisher(UInt32, 'shooter', 10)
        self.save_sub = self.create_subscription(Int8, "data_save", self.save_callback, 10)
        self.file = f'/home/{username}/rabbit_ws/src/rabbit_shooter/config/data.yaml'
        self.file1 = f'/home/{username}/rabbit_ws/src/rabbit_shooter/config/data1.yaml'
        self.file2= f'/home/{username}/rabbit_ws/src/rabbit_shooter/config/data2.yaml'

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
            left = self.map(self.adjust_left, 1, -1, 0 ,0.02)
            right = self.map(self.adjust_right, 1, -1, 0, 0.02)
            self.adjust = self.distance + (right - left)
            if (self.adjust <0):
                self.adjust = 0
            self.speed = self.speed_shooter(self.adjust)
            self.rps = self.map(self.speed, 0, 1500, 0, 65535)
            
            
            
            shooter_msg = UInt32()
            shooter_msg.data = int(self.rps)
            self.shooter_pub.publish(shooter_msg)
        
            

    def save_callback(self, save_msg):
        self.save = save_msg.data
        count = str(self.count)
        if(self.save == 1 and self.push_save == 0):
            self.push_save = 1
            if(self.saved == 0):
                self.saved = 1
                if(self.distance>=0.2 and self.distance<=0.8):
                    name = str(self.count)
                    read_and_modify_one_block_of_yaml_data(self.file, key=f'point_{self.count}', value=[self.distance, self.speed])
                    self.save = 0
                    print("saved")
                    print(self.count,self.distance, self.speed)
                    self.count = self.count + 1
                    read_and_modify_one_block_of_yaml_data(self.file, key=f'len_i', value = self.count)
                elif(self.distance>=1.4 and self.distance<=2.2):
                    name = str(self.count)
                    read_and_modify_one_block_of_yaml_data(self.file1, key=f'point_{self.count}', value=[self.distance, self.speed])
                    self.save = 0
                    print("saved")
                    print(self.count,self.distance, self.speed)
                    self.count = self.count + 1
                    read_and_modify_one_block_of_yaml_data(self.file2, key=f'len_i', value = self.count)
                elif(self.distance>0.8 and self.distance<1.4):
                    name = str(self.count)
                    read_and_modify_one_block_of_yaml_data(self.file2, key=f'point_{self.count}', value=[self.distance, self.speed])
                    self.save = 0
                    print("saved")
                    print(self.count,self.distance, self.speed)
                    self.count = self.count + 1
                    read_and_modify_one_block_of_yaml_data(self.file2, key=f'len_i', value = self.count)


        elif(self.save == 0 and self.push_save == 1):
            self.push_save = 0
            self.saved = 0

    def speed_shooter(self, x):
        if(x >= 0.2 and x<=0.8): # Pole 1meter
            y = 78.61 * x + 468.9
        elif(x >= 1.4 and x<= 2.2 ): # Pole 1.5 meter
            y = 102.8*x + 718.6
        elif(x > 0.8 and x<1.4): # Pole 1meter on the next side
            y = 172.3*x + 479.5
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
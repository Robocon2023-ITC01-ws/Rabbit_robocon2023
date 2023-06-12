#!/usr/bin/env python3
import numpy as np
import getpass
import yaml
username = getpass.getuser()

def read_one_block_of_yaml_data(filename):
    with open(f'{filename}','r') as f:
        output = yaml.safe_load(f) 
    return output['distance']


class shooter():
    def __init__(self, distance):
        self.distance = distance
        self.param = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0]
        self.file = f'/home/{username}/rabbit_ws/src/rabbit_shooter/config/param.yaml'
        self.param = read_one_block_of_yaml_data(self.file)
        print(self.param)
        
        # Parameters
        self.d_pulley_1 = 0.016
        self.d_pulley_2 = 0.04
        self.d_roller = 0.04
        self.shooter_h = 0.42

    def velocity_to_can(self, velocity):
        rps2 = velocity*2/(self.d_roller)
        rps1 = rps2*(self.d_pulley_2/self.d_pulley_1)
        v_to_roller = rps1
        if (v_to_roller > 1500):
            v_in = 1500
        v_o = (int)(self.map(v_to_roller, 0, 1500, 0, 65535))
        return v_o
    
    def velocity(self, distance):
        theta = 60.5*np.pi/180
        h = 0.0
        if(distance > 3.3-0.2 and distance<=3.3+0.2):
            h = -0.8 + 0.42
        elif(distance >=1.2-0.2 and distance<1.2+0.2):
            h = -1.0 + 0.42
        elif(distance >=3.0-0.2 and distance<3.0+0.2):
            h = -1.0 + 0.42
        elif(distance >=1.4-0.15 and distance <=1.4+0.15):
            h = -1.7 + 0.42
        v = np.sqrt(9.8*self.distance**2/(self.distance*np.tan(theta) - h))
        return v


    def map(self, Input, Min_Input, Max_Input, Min_Output, Max_Output):
        value =  ((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output)
        return value
    
    def shooter(self):
        X_in = self.distance
        if (X_in>=0.9 and X_in<1):
            X_in = X_in*self.param[0]
            #X_in = X_in*3.1
        elif(X_in>=1 and X_in <=1.05):
            X_in = X_in*self.param[1]
            #X_in = X_in*3.25
        elif(X_in>1.05 and X_in <=1.1):
            X_in = X_in*self.param[2]
            #X_in = X_in*3.05
        elif (X_in>1.1 and X_in<=1.2):
            X_in = X_in*self.param[3]
            #X_in = X_in*2
        elif(X_in>1.2 and X_in<=1.3):
            X_in = X_in*self.param[4]
            #X_in = X_in*1.9
        elif(X_in>1.3 and X_in<=1.4):
            X_in = X_in*self.param[5]
            #X_in = X_in * 2.1
        elif(X_in> 1.4 and X_in<=1.5):
            a = self.map(X_in, 1.5, 0.5, self.param[6], 1.5)
            X_in = a
            #X_in = self.map(X_in, 1.5, 0.5, 3.5, 1.5)
        elif(X_in>1.5 and X_in <= 1.7):
            a = self.map(X_in, 1.5, 0.5, self.param[7], 1.5)
            X_in = a
            #X_in = self.map(X_in, 1.5, 0.5, 2.95, 1.5)
        elif(X_in>1.7 and X_in<=1.8):
            a = self.map(X_in, 1.5, 0.5, self.param[8], 1.5)
            X_in = a
            #X_in = self.map(X_in, 1.5, 0.5, 2.94, 1.5)
        elif(X_in>1.8 and X_in <=2):
            a = self.map(X_in, 1.5, 0.5, self.param[9], 1.5)
            X_in = a
            #X_in = self.map(X_in, 1.5, 0.5, 3.12, 1.5)
        elif(X_in>2 and X_in<2.2):
            a = self.map(X_in, 1.5, 0.5, self.param[10], 1.5)
            X_in = a
            #X_in = self.map(X_in, 1.5, 0.5, 3.1, 1.5)
        elif(X_in>=2.2 and X_in<=2.3):
            a = self.map(X_in, 1.5, 0.5, self.param[11], 1.5)
            X_in = a
            #X_in = self.map(X_in, 1.5, 0.5, 3.09, 1.5)
        elif(X_in>2.3 and X_in<=2.5):
            a = self.map(X_in, 1.5, 0.5, self.param[12], 1.5)
            X_in = a
            #X_in = self.map(X_in, 1.5, 0.5, 3.7, 1.5)
        elif(X_in>2.5 and X_in<=2.6):
            a = self.map(X_in, 1.5, 0.5, self.param[13], 1.5)
            X_in = a
            #X_in = self.map(X_in, 1.5, 0.5, 3.73, 1.5)    
        elif(X_in>2.6 and X_in<=2.7):
            a = self.map(X_in, 1.5, 0.5, self.param[14], 1.5)
            X_in = a
            #X_in = self.map(X_in, 1.5, 0.5, 2.8, 1.5)
        elif(X_in>2.7 and X_in<=2.8):
            a = self.map(X_in, 1.5, 0.5, self.param[15], 1.5)
            X_in = a
            #X_in = self.map(X_in, 1.5, 0.5, 2.7, 1.5) 
        elif(X_in>2.8 and X_in<=2.9):
            a = self.map(X_in, 1.5, 0.5, self.param[16], 1.5)
            X_in = a
            #X_in = self.map(X_in, 1.5, 0.5, 2.65, 1.5) 
        elif(X_in>3.2 and X_in<=3.5):
            a = self.map(X_in, 1.5, 0.5, self.param[17], 1.5)
            X_in = a
            #X_in = self.map(X_in, 1.5, 0.5, 2.65, 1.5)
        elif(X_in>3.5 and X_in<=3.8):
            a = self.map(X_in, 1.5, 0.5, self.param[18], 1.5)
            X_in = a
            #X_in = self.map(X_in, 2.25, 0.5, 3.9, 1.5)
        elif(X_in>3.8 and X_in<=4.1):
            a = self.map(X_in, 1.5, 0.5, self.param[19], 1.5)
            X_in = a
            #X_in = self.map(X_in, 2.35, 0.5, 4.05, 1.5)
        elif(X_in>4.1 and X_in<=4.4):
            a = self.map(X_in, 1.5, 0.5, self.param[20], 1.5)
            X_in = a
            #X_in = self.map(X_in, 2.46, 0.5, 4.15, 1.5)
        elif(X_in>=5.5 and X_in<5.6):
            a = self.map(X_in, 1.5, 0.5, self.param[21], 1.5)
            X_in = a
            #X_in = self.map(X_in,1.5, 0.5,3.5, 1.5)
        elif(X_in>0.7 and X_in<0.9):
            X_in = X_in*self.param[22]
            #X_in = 3.4*X_in 
        elif(X_in>0.6 and X_in<=0.7):
            X_in = X_in*self.param[23]
            #X_in = 3.9*X_in
        else:
            X_in = X_in*self.param[24]
        self.distance = X_in
        v = self.velocity(X_in)
        
        rps = self.velocity_to_can(v)
        return rps


    

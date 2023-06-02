#!/usr/bin/env python3
import numpy as np

class shooter():
    def __init__(self, distance):
        self.distance = distance
        
        # Parameters
        self.d_pulley_1 = 0.016
        self.d_pulley_2 = 0.04
        self.d_roller = 0.04
        self.shooter_h = 0.4

    def velocity_to_can(self, velocity):
        rps2 = velocity*2/(self.d_roller)
        rps1 = rps2*(self.d_pulley_2/self.d_pulley_1)
        v_to_roller = rps1
        if (v_to_roller > 1500):
            v_in = 1500
        v_o = (int)(self.map(v_to_roller, 0, 1500, 0, 65535))
        return v_o
    
    def velocity(self, distance):
        theta = np.pi/4
        h = 0.0
        if(distance > 0.6 and distance<=1.7):
            h = -1 + self.shooter_h
        elif(distance >=2.5 and distance<3.8):
            h = -1.2 + self.shooter_h
            distance = distance * 4
        elif(distance >=3.8 and distance <=4.1):
            h = -1.9 + self.shooter_h
            distance = distance * 4.5
        v = np.sqrt(9.8*self.distance**2/(self.distance*np.tan(theta) - h))
        return v

    def map(self, Input, Min_Input, Max_Input, Min_Output, Max_Output):
        value =  ((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output)
        return value
    
    def shooter(self):
        X_in = self.distance
        if (X_in>=0.9 and X_in<1):
            X_in = X_in*3.1
        elif(X_in>=1 and X_in <=1.05):
            X_in = X_in*3.25
        elif(X_in>1.05 and X_in <=1.1):
            X_in = X_in*3.05
        elif (X_in>1.1 and X_in<=1.2):
            X_in = X_in*1.9
        elif(X_in>1.2 and X_in<=1.3):
            X_in = X_in*2.35
        elif(X_in>1.3 and X_in<=1.4):
            X_in = X_in * 2.42
        elif(X_in> 1.4 and X_in<=1.5):
            X_in = self.map(X_in, 1.5, 0.5, 3.5, 1.5)
        elif(X_in>1.5 and X_in <= 1.7):
            X_in = self.map(X_in, 1.5, 0.5, 2.95, 1.5)
        elif(X_in>1.7 and X_in<=1.8):
            X_in = self.map(X_in, 1.5, 0.5, 2.94, 1.5)
        elif(X_in>1.8 and X_in <=2):
            X_in = self.map(X_in, 1.5, 0.5, 3.12, 1.5)
        elif(X_in>2 and X_in<2.2):
            X_in = self.map(X_in, 1.5, 0.5, 3.1, 1.5)
        elif(X_in>=2.2 and X_in<=2.3):
            X_in = self.map(X_in, 1.5, 0.5, 3.09, 1.5)
        elif(X_in>2.3 and X_in<=2.5):
            X_in = self.map(X_in, 1.5, 0.5, 3.7, 1.5)
        elif(X_in>2.5 and X_in<=2.6):
            X_in = self.map(X_in, 1.5, 0.5, 3.73, 1.5)    
        elif(X_in>2.6 and X_in<=2.7):
            X_in = self.map(X_in, 1.5, 0.5, 2.8, 1.5)
        elif(X_in>2.7 and X_in<=2.8):
            X_in = self.map(X_in, 1.5, 0.5, 2.7, 1.5) 
        elif(X_in>2.8 and X_in<=2.9):
            X_in = self.map(X_in, 1.5, 0.5, 2.65, 1.5) 
        elif(X_in>3.2 and X_in<=3.5):
            X_in = self.map(X_in, 1.5, 0.5, 2.65, 1.5)
        elif(X_in>3.5 and X_in<=3.8):
            X_in = self.map(X_in, 2.25, 0.5, 3.9, 1.5)
        elif(X_in>3.8 and X_in<=4.1):
            X_in = self.map(X_in, 2.35, 0.5, 4.05, 1.5)
        elif(X_in>4.1 and X_in<=4.4):
            X_in = self.map(X_in, 2.46, 0.5, 4.15, 1.5)
        elif(X_in>=5.5 and X_in<5.6):
            X_in = self.map(X_in,1.5, 0.5,3.5, 1.5)
        elif(X_in>0.7 and X_in<0.9):
            X_in = 3.73*X_in 
        elif(X_in>0.6 and X_in<=0.7):
            X_in = 3.9*X_in
        else:
            X_in = 3.5*X_in
        self.distance = X_in
        v = self.velocity(X_in)
        
        rps = self.velocity_to_can(v)
        return rps


    

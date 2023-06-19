#!/usr/bin/env python3
import numpy as np

class shooter():
    def __init__(self, distance, adjust, data):
        self.distance = distance
        self.adjust = adjust

        self.data = data
        # Parameters
        self.d_pulley_1 = 0.016
        self.d_pulley_2 = 0.04
        self.d_roller = 0.04
        self.a = np.zeros(25)

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
        a= self.data
        X_in = self.distance
        a = np.array([a[0], a[1],a[2], a[3], a[4],
                    a[5], a[6],a[7], a[8], a[9],
                    a[10], a[11],a[12], a[13], a[14],
                    a[15], a[16],a[17], a[18], a[19],
                    a[20], a[21],a[22], a[23], a[24],
                    ])
        if (X_in>=0.9 and X_in<1):
            a[0] = self.map(self.adjust, 1,-1,3,3.4)
            X_in = X_in*a[0]
        elif(X_in>=1 and X_in <=1.1):
            a[1]= self.map(self.adjust, 1, -1, 3,3.42)
            X_in = X_in*a[1]
        elif(X_in>1.1 and X_in <=1.2):
            a[2] = self.map(self.adjust, 1, -1, 3,3.43)
            X_in = X_in*a[2]
        elif (X_in>1.2 and X_in<=1.4):
            a[3] = self.map(self.adjust, 1, -1, 3, 3.45)
            X_in = X_in*a[3]
        elif(X_in>1.3 and X_in<=1.5):
            a[4] = self.map(self.adjust, 1, -1, 3,3.5)
            X_in = X_in*a[4]
        elif(X_in>1.4 and X_in<=1.6):
            a[5]= self.map(self.adjust, 1, -1, 3,3.5)
            X_in = X_in*a[5]
        elif(X_in> 1.6 and X_in<=1.7):
            a[6] =self.map(self.adjust, 1, -1, 3, 10)
            X_in = self.map(X_in, 1.5, 0.5, a[6], 1.5)
            #X_in = self.map(X_in, 1.5, 0.5, 3.5, 1.5)
        elif(X_in>1.7 and X_in <= 1.8):
            a[7] =self.map(self.adjust, 1, -1, 3, 11)
            X_in = self.map(X_in, 1.5, 0.5, a[7], 1.5)
            #X_in = self.map(X_in, 1.5, 0.5, 2.95, 1.5)
        elif(X_in>1.8 and X_in<=1.9):
            a[8] = self.map(self.adjust, 1, -1, 3, 12)
            X_in = self.map(X_in, 1.5, 0.5, a[8], 1.5)
            #X_in = self.map(X_in, 1.5, 0.5, 2.94, 1.5)
        elif(X_in>1.9 and X_in <=2.0):
            a[9] = self.map(self.adjust, 1, -1, 3, 12)
            X_in = self.map(X_in, 1.5, 0.5, a[9], 1.5)
            #X_in = self.map(X_in, 1.5, 0.5, 3.12, 1.5)
        elif(X_in>2.0 and X_in<2.1):
            a[10] = self.map(self.adjust, 1, -1, 3, 12)
            X_in = self.map(X_in, 1.5, 0.5, a[10], 1.5)
            #X_in = self.map(X_in, 1.5, 0.5, 3.1, 1.5)
        elif(X_in>=2.1 and X_in<=2.2):
            a[11] = self.map(self.adjust, 1, -1, 3, 3.15)
            X_in = self.map(X_in, 1.5, 0.5, a[11], 1.5)
            #X_in = self.map(X_in, 1.5, 0.5, 3.09, 1.5)
        elif(X_in>2.2 and X_in<=2.3):
            a[12] = self.map(self.adjust, 1, -1, 3.2, 3)
            X_in = self.map(X_in, 1.5, 0.5, a[12], 1.5)
            #X_in = self.map(X_in, 1.5, 0.5, 3.7, 1.5)
        elif(X_in>2.3 and X_in<=2.4):
            a[13] = self.map(self.adjust, 1, -1, 3.6, 3.75)
            X_in = self.map(X_in, 1.5, 0.5, a[13], 1.5)
            #X_in = self.map(X_in, 1.5, 0.5, 3.73, 1.5)    
        elif(X_in>2.4 and X_in<=2.5):
            a[14] = self.map(self.adjust, 1, -1, 3.3, 3.4)
            X_in = self.map(X_in, 1.5, 0.5, a[14], 1.5)
            #X_in = self.map(X_in, 1.5, 0.5, 2.8, 1.5)
        elif(X_in>2.5 and X_in<=2.6):
            a[15] = self.map(self.adjust, 1, -1, 3.8, 3.9)
            X_in = self.map(X_in, 1.5, 0.5, a[15], 1.5)
            #X_in = self.map(X_in, 1.5, 0.5, 2.7, 1.5) 
        elif(X_in>2.6 and X_in<=2.7):
            a[16] = self.map(self.adjust, 1, -1, 3.5, 4)
            X_in = self.map(X_in, 1.5, 0.5, a[16], 1.5)
            
            #X_in = self.map(X_in, 1.5, 0.5, 2.65, 1.5) 
        elif(X_in>2.7 and X_in<=2.8):
            a[17] = self.map(self.adjust, 1, -1, 3.5, 4)
            X_in = self.map(X_in, 1.5, 0.5, a[17], 1.5)
            #X_in = self.map(X_in, 1.5, 0.5, 2.65, 1.5)
        elif(X_in>2.8 and X_in<=2.9):
            a[18] = self.map(self.adjust, 1, -1, 3.87, 3.92)
            X_in = self.map(X_in, 1.5, 0.5, a[18], 1.5)
            

            #X_in = self.map(X_in, 2.25, 0.5, 3.9, 1.5)
        elif(X_in>2.9 and X_in<=3.0):
            a[17] = self.map(self.adjust, 1, -1, 4, 4.1)
            X_in = self.map(X_in, 1.5, 0.5, a[17], 1.5)

            #X_in = self.map(X_in, 2.35, 0.5, 4.05, 1.5)
        elif(X_in>.1 and X_in<=4.4):
            a[20] = self.map(self.adjust, 1, -1, 3.8, 3.9)
            X_in = self.map(X_in, 1.5, 0.5, a[20], 1.5)

            #X_in = self.map(X_in, 2.46, 0.5, 4.15, 1.5)
        elif(X_in>=5.5 and X_in<5.6):
            a[21] = self.map(self.adjust, 1, -1, 3.3, 3.7)
            X_in = self.map(X_in, 1.5, 0.5, a[21], 1.5)

            #X_in = self.map(X_in,1.5, 0.5,3.5, 1.5)
        elif(X_in>0.7 and X_in<0.9):
            a[22] = self.map(self.adjust, 1,-1,3.5,3.6)
            X_in = X_in*a[22]
            

            #X_in = 3.4*X_in 
        elif(X_in>0.6 and X_in<=0.7):
            a[23] = self.map(self.adjust, 1,-1,3.88,3.92)
            X_in = X_in*a[23]

            #X_in = 3.9*X_in
        else:
            a[24] = self.map(self.adjust, 1,-1,3.47,3.52)
            X_in = X_in*a[24]

            #X_in = 3.5*X_in
        self.distance = X_in
        self.x = np.array([a[0], a[1],a[2], a[3], a[4],
                                    a[5], a[6],a[7], a[8], a[9],
                                    a[10], a[11],a[12], a[13], a[14],
                                    a[15], a[16],a[17], a[18], a[19],
                                    a[20], a[21],a[22], a[23], a[24],
                                    ])

        v = self.velocity(X_in)
       
        rps = self.velocity_to_can(v)
        return rps, X_in, self.x


    
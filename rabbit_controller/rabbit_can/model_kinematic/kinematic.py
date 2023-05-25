import math
import numpy as np

class kinematic():
    def __init__(self):
        
        self.R = 0.245 # [m]
        self.r = 0.06 # [m]
        self.a1 = np.pi/4
        self.a2 = 3*np.pi/4
        self.a3 = 5*np.pi/4
        self.a4 = 7*np.pi/4

        
    def omni_inverse_kinematic(self, vx, vy, vth, theta):

        J_inv = (1/self.r)*np.array([
            [np.sin(theta+self.a1), np.cos(theta+self.a1), self.R],
            [-np.sin(theta+self.a2), -np.cos(theta+self.a2), self.R],
            [np.sin(theta+self.a3), np.cos(theta+self.a3), self.R],
            [-np.sin(theta+self.a4), -np.cos(theta+self.a4), self.R],
        ])

        # J_inv = (1/self.r)*np.array([
        #     [np.sin(self.a1), np.cos(self.a1), self.R],
        #     [-np.sin(self.a2), -np.cos(self.a2), self.R],
        #     [np.sin(self.a3), np.cos(self.a3), self.R],
        #     [-np.sin(self.a4), -np.cos(self.a4), self.R],
        # ])

        inv_vec = J_inv@np.array([vx, vy, vth])
	
        return inv_vec[0], inv_vec[1], inv_vec[2], inv_vec[3]

    def omni_forward_kinematic(self, w1, w2, w3, w4, theta):

        J_for = (self.r/2)*np.array([
            [np.sin(theta+self.a1), -np.sin(theta+self.a2), np.sin(theta+self.a3), -np.sin(theta+self.a4)],
            [np.cos(theta+self.a1), -np.cos(theta+self.a2), np.cos(theta+self.a3), -np.cos(theta+self.a4)],
            [1/(2*0.22), 1/(2*0.22), 1/(2*0.22), 1/(2*0.22)],
        ])

        vec = J_for@np.array([w1, w2, w3, w4])

        return vec[0], vec[1], vec[2]

    def map(self,Input, min_input, max_input, min_output, max_output):
        value = ((Input - min_input)*(max_output-min_output)/(max_input - min_input) + min_output)
        return value

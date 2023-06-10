import math
import numpy as np

class kinematic():
    def __init__(self):
        
        self.r = 0.0475 
        self.lx = 0.1925
        self.ly = 0.1775

        
    def meca_inverse_kinematic(self, vx, vy, vth, theta):
            
        J_inv = (1/self.r)*np.array([
            [-1, 1, -1, 1],
            [1, 1, -1, -1],
            [-(self.lx+self.ly), (self.lx+self.ly), -(self.lx+self.ly), (self.lx+self.ly)]
        ], dtype=np.float64).T

        inv_vec = J_inv@np.array([vx, vy, vth])
	
        return inv_vec[0], inv_vec[1], inv_vec[2], inv_vec[3]

    def meca_forward_kinematic(self, w1, w2, w3, w4, theta):
        rot_mat = np.array([
                           [np.cos(theta), np.sin(theta), 0],
                           [-np.sin(theta), np.cos(theta), 0],
                           [0, 0, 1]], dtype=np.float64)
        J_for = (self.r/4)*np.array([
            [-1, 1, -1, 1],
            [1, 1, -1, -1],
            [-1/(self.lx+self.ly), 1/(self.lx+self.ly), -1/(self.lx+self.ly), 1/(self.lx+self.ly)]
        ], dtype=np.float64)

        vec = rot_mat.T@J_for@np.array([w1, w2, w3, w4])

        return vec[0], vec[1], vec[2]

    def map(self,Input, min_input, max_input, min_output, max_output):
        value = ((Input - min_input)*(max_output-min_output)/(max_input - min_input) + min_output)
        return value

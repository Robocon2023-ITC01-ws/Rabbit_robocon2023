import math
import numpy as np

class kinematic():
    def __init__(self):
        self.wheel_radius = 0.05 # [m]
        self.Lxl = 0.21 # [m]
        self.Ly = 0.20 # [m]


    def mecanum_inverse_kinematic(self, vx, vy, vth):
        J_inv = (1/self.wheel_radius)*np.array([
		[1, 1, (self.Lxl + self.Ly)],
		[-1, 1, (self.Lxl + self.Ly)],
		[-1, -1, (self.Lxl + self.Ly)],
		[1,-1,  (self.Lxl + self.Ly)]])  
        inv_vec = J_inv @ np.array([vx, vy, vth])
	
        return inv_vec[0], inv_vec[1], inv_vec[2], inv_vec[3]

    def mecanum_forward_kinematic(self, w1, w2, w3, w4):
        J = (self.wheel_radius/4)*np.array([
            [1, -1, -1, 1],
            [1, 1, -1, -1],
            [w1/(self.Lxl + self.Ly), w2/(self.Lxl + self.Ly), w3(self.Lxl + self.Ly),w4/(self.Lxl + self.Ly)]
        ])
        vec = J @ np.array([w1, w2, w3, w4])
        return vec[0], vec[1], vec[2]

    def map(self,Input, min_input, max_input, min_output, max_output):
        value = ((Input - min_input)*(max_output-min_output)/(max_input - min_input) + min_output)
        return value

    
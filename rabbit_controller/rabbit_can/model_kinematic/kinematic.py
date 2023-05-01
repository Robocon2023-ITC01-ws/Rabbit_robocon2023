import math
import numpy as np

class kinematic():
    def __init__(self):
        
        self.R = 0.22 # [m]
        self.r = 0.05 # [m]


    # def omni_inverse_kinematic(self, vx, vy, vth):
    #     J_inv = (1/self.r)*np.array([
	# 	[np.sin(np.pi/4), np.cos(np.pi/4), self.R],
	# 	[np.sin(3*np.pi/4),np.cos(3*np.pi/4),self.R],
	# 	[-np.sin(5*np.pi/4), np.cos(5*np.pi/4), -self.R],
	# 	[-np.sin(7*np.pi/4), np.cos(7*np.pi/4),  -self.R]])  
    #     inv_vec = J_inv @ np.array([vx, vy, vth])
	
    #     return inv_vec[0], inv_vec[1], inv_vec[2], inv_vec[3]

    # def omni_forward_kinematic(self, w1, w2, w3, w4):
    #     J = (self.r/2)*np.array([
	# 	[np.sin(np.pi/4), np.cos(np.pi/4), 1/(2*self.R)],
	# 	[np.sin(3*np.pi/4),np.cos(3*np.pi/4), 1/(2*self.R)],
	# 	[-np.sin(5*np.pi/4), np.cos(5*np.pi/4), -1/(2*self.R)],
	# 	[-np.sin(7*np.pi/4), np.cos(7*np.pi/4),  -1/(2*self.R)]])
    #     vec = J.T @ np.array([w1, w2, w3, w4])
    #     return vec[0], vec[1], vec[2]
    def omni_inverse_kinematic(self, vx, vy, vth, theta):
        J_inv = (1/self.r)*np.array([
            [np.sin(theta+np.pi/4),   np.cos(theta+np.pi/4), 0.22],
            [-np.sin(theta+3*np.pi/4), -np.cos(theta+3*np.pi/4), 0.22],
            [np.sin(theta+5*np.pi/4), np.cos(theta+5*np.pi/4), 0.22],
            [-np.sin(theta+7*np.pi/4), -np.cos(theta+7*np.pi/4), 0.22]]) 
        rot_mat = np.array([
            [np.cos(theta), np.sin(theta), 0],
            [-np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
        ]) 
        inv_vec = J_inv @ rot_mat@ np.array([vx, vy, vth])
	
        return inv_vec[0], inv_vec[1], inv_vec[2], inv_vec[3]

    def omni_forward_kinematic(self, w1, w2, w3, w4, theta):
        J = (self.r/2)*np.array([
		[math.sin(theta+math.pi/4), -math.sin(theta+3*math.pi/4), math.sin(theta+5*math.pi/4), -math.sin(theta+7*math.pi/4)],
        [ math.cos(theta+math.pi/4),  -math.cos(theta+3*math.pi/4),  math.cos(theta+5*math.pi/4),  -math.cos(theta+7*math.pi/4)],
        [1/(2*0.22), 1/(2*0.22), 1/(2*0.22), 1/(2*0.22)]])
        rot_mat = np.array([
    		[np.cos(theta), np.sin(theta), 0],
    		[-np.sin(theta), np.cos(theta), 0],
    		[0, 0, 1]])
        #vec = rot_mat.T @J @ np.array([w1, w2, w3, w4])
        vec = rot_mat.T@J @ np.array([[w1], [w2], [w3], [w4]])
        return vec[0], vec[1], vec[2]

    def map(self,Input, min_input, max_input, min_output, max_output):
        value = ((Input - min_input)*(max_output-min_output)/(max_input - min_input) + min_output)
        return value

    

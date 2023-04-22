import math
import casadi as ca
import numpy as np

class RabbitModel():
    def __init__(self):

        # Kinematic Configuration
        self.wheel_radius = 0.05 # [m]
        self.Lx = 0.21           # [m]
        self.Ly = 0.20           # [m]
        self.J_inv = (1/self.wheel_radius) * np.array([
            [1, 1, -(self.Lx+self.Ly)],
            [-1,  1,  (self.Lx+self.Ly)],
            [-1,  -1, (self.Lx+self.Ly)],
            [1, -1,  -(self.Lx+self.Ly)]
        ])

    def forward_matrix(self, type=None):
        if type=="numpy":
            J_for = (self.wheel_radius/4)*np.array([
                [1,  1, -1,  -1],
                [1, -1, -1, 1],
                [-1/(self.Lx+self.Ly), 1/(self.Lx+self.Ly), 1/(self.Lx+self.Ly), -1/(self.Lx+self.Ly)]
            ])
        elif type=="sym":
            J_for = (self.wheel_radius/4)*ca.DM([
            [1,  1, -1,  -1],
            [1, -1, -1, 1],
            [-1/(self.Lx+self.Ly), 1/(self.Lx+self.Ly), 1/(self.Lx+self.Ly), -1/(self.Lx+self.Ly)]
        ])
        return J_for

    def inverse_matrix(self, type):
        if type=="sym":
            J_inv = (1/self.wheel_radius) * ca.DM([
            [1, 1, -(self.Lx+self.Ly)],
            [-1,  1,  (self.Lx+self.Ly)],
            [-1,  -1, (self.Lx+self.Ly)],
            [1, -1,  -(self.Lx+self.Ly)]
        ])

        elif type=="numpy":
            J_inv = (1/self.wheel_radius) * np.array([
            [1, 1, -(self.Lx+self.Ly)],
            [-1,  1,  (self.Lx+self.Ly)],
            [-1,  -1, (self.Lx+self.Ly)],
            [1, -1,  -(self.Lx+self.Ly)]
        ])

        return J_inv

    def rotation_matrix(self, angle, type=None):
        if type=="sym":
            rot = ca.vertcat(
                ca.horzcat(ca.cos(angle), ca.sin(angle), 0),
                ca.horzcat(-ca.sin(angle), ca.cos(angle), 0),
                ca.horzcat(0, 0, 1)
            )
        if type=="numpy":
            rot = np.array([
            [ math.cos(angle),  math.sin(angle),   0],
            [-math.sin(angle),  math.cos(angle),   0],
            [0             ,              0,       1]
        ])
        return rot

    def forward_kinematic(self, v1, v2, v3, v4, angle, type=None):
        if type=="sym":
            vec_for = self.rotation_matrix(angle, type).T@self.forward_matrix(type)@ca.vertcat(v1, v2, v3, v4)
        elif type=="numpy":
            vec_for = self.rotation_matrix(angle, type).T@self.forward_matrix(type)@np.array([v1, v2, v3, v4])
        return vec_for

    def inverse_kinematic(self, vx, vy, vth,angle, type):
        if type=="sym":
            vec_inv = self.rotation_matrix(angle, type)@self.inverse_matrix(type)@ca.vertcat(vx, vy, vth)
        elif type=="numpy":
            vec_inv = self.rotation_matrix(angle, type)@self.inverse_matrix(type)@np.array([vx, vy, vth])
        return vec_inv

    def velocity_from_discrete_points(self, k, dt, x_ref, y_ref, theta_ref):
        vx = (x_ref[k]-x_ref[k-1])/dt
        vy = (y_ref[k]-y_ref[k-1])/dt
        vth = (theta_ref[k]-theta_ref[k-1])/dt

        return vx, vy, 

    def map(self,Input, min_input, max_input, min_output, max_output):
        value = ((Input - min_input)*(max_output-min_output)/(max_input - min_input) + min_output)
        return value

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return qx, qy, qz, qw

    def euler_from_quaternion(self, x, y, z, w):
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
		
        t2 = 2.0 * (w * y - z * x)
        t2 = 1.0 if t2 > 1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
		
        t3 = 2.0 * (w * z +x * y)
        t4 = 1.0 - 2.0*(y * y + z * z)
        yaw = math.atan2(t3, t4)
		
		#if yaw < 0:
		#	yaw = self.map(yaw, -3.1399, -0.001, 3.1399, 6.2799)
		
        return roll, pitch, yaw

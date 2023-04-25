import numpy as np
import math
import casadi as ca


class RabbitModel():

    def __init__(self):

        # Kinematic Configuration
        self.wheel_radius = 0.05 # [m]
        self.Lx = 0.17           # [m]
        self.Ly = 0.13          # [m]
    def forward_matrix(self, type=None):
        if type=="numpy":
            J_for = (self.wheel_radius/4)*np.array([
                [1,  1, -1,  -1],
                [1,  -1, -1, 1],
                [-1/(self.Lx+self.Ly), 1/(self.Lx+self.Ly), 1/(self.Lx+self.Ly), -1/(self.Lx+self.Ly)]
            ])
        elif type=="sym":
            J_for = (self.wheel_radius/4)*ca.DM([
            [1,  1, -1,  -1],
            [1,  -1, -1,  1],
            [-1/(self.Lx+self.Ly), 1/(self.Lx+self.Ly), 1/(self.Lx+self.Ly), -1/(self.Lx+self.Ly)]
        ])
        return J_for

    def inverse_matrix(self, type):
        if type=="sym":
            J_inv = (1/self.wheel_radius) * ca.DM([
            [ 1,   1,  (self.Lx+self.Ly)],
            [-1,  1,  (self.Lx+self.Ly)],
            [-1, -1, (self.Lx+self.Ly)],
            [1,  -1, (self.Lx+self.Ly)]
        ])

        elif type=="numpy":
            J_inv = (1/self.wheel_radius) * np.array([
            [ 1,   1,  (self.Lx+self.Ly)],
            [-1,  1,  (self.Lx+self.Ly)],
            [-1, -1, (self.Lx+self.Ly)],
            [1,  -1, (self.Lx+self.Ly)]
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

    def inverse_kinematic(self, vx, vy, vth, type):
        if type=="sym":
            vec_inv = self.inverse_matrix(type)@ca.vertcat(vx, vy, vth)
        elif type=="numpy":
            vec_inv = self.inverse_matrix(type)@np.array([vx, vy, vth])
        return vec_inv

    def velocity_from_discrete_points(self, k, dt, x_ref, y_ref, theta_ref):
        vx = (x_ref[k]-x_ref[k-1])/dt
        vy = (y_ref[k]-y_ref[k-1])/dt
        vth = (theta_ref[k]-theta_ref[k-1])/dt

        return vx, vy, vth

    def inv_accelerate_from_discrete_points(self, k, dt, w1, w2, w3, w4):
        a1 = (w1[k]-w1[k-1])/dt
        a2 = (w2[k]-w2[k-1])/dt
        a3 = (w3[k]-w3[k-1])/dt
        a4 = (w4[k]-w4[k-1])/dt
        return a1, a2, a3, a4

    def dynamic_model(self, index, dt,
        theta,
        vx, vy, vth,
        v1, v2, v3, v4,
        a1, a2, a3, a4,
        dyna_type=None
    ):
        if dyna_type=="derive":
            forward = self.rotation_matrix(theta, "sym").T@ca.vertcat(vx, vy, vth)
            accel_x_1 = ((-ca.sin(theta)-ca.cos(theta))*(v2+v3)+(-ca.sin(theta)+ca.cos(theta))*(v1+v4))*vth
            accel_x_2 = (-ca.sin(theta)+ca.cos(theta))*(a2+a3)+(ca.sin(theta)+ca.cos(theta))*(a1+a4)
            accel_x = (self.wheel_radius/4)*(accel_x_1+accel_x_2)
            accel_y_1 = ((-ca.sin(theta)+ca.cos(theta))*(v2+v3)+(ca.sin(theta)+ca.cos(theta))*(v1+v4))*vth
            accel_y_2 = (ca.sin(theta)+ca.cos(theta))*(a2+a3)+(ca.sin(theta)-ca.cos(theta))*(a1+a4)
            accel_y = (self.wheel_radius/4)*(accel_y_1+accel_y_2)
            accel_w = (self.wheel_radius/4)*(-a1+a2-a3+a4)*(1/(self.Lx+self.Ly))

            return ca.vertcat(accel_x, accel_y, accel_w)

        if dyna_type=="test1":

            accel_x = (vx[index]-vx[index-1])/dt
            accel_y = (vy[index]-vy[index-1])/dt
            accel_w = (vth[index]-vth[index-1])/dt

            return ca.vertcat(vx, vy, vth)


    def forward_dynamic(self):
        pass

    def inverse_dynamic(self):
        pass

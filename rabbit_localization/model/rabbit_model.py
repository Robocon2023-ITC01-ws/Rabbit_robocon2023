import numpy as np
import math
import casadi as ca


class RabbitModel():

    def __init__(self):

        # Kinematic Configuration
        self.R = 0.22
        self.r = 0.06
        self.a1 = math.pi/4
        self.a2 = 3*math.pi/4
        self.a3 = 5*math.pi/4
        self.a4 = 7*math.pi/4
    def forward_matrix(self, theta, type=None):
        if type=="numpy":
            J_for = (self.r/2)*np.array([
                [math.sin(theta+self.a1), math.sin(theta+self.a2), -math.sin(theta+self.a3), -math.sin(theta+self.a4)],
                [ math.cos(theta+self.a1),  math.cos(theta+self.a2),  -math.cos(theta+self.a3),  -math.cos(theta+self.a4)],
                [1/(2*0.22), -1/(2*0.22), -1/(2*0.22), 1/(2*0.22)]
            ], dtype=np.float32)
        elif type=="sym":
            J_for = (self.r/2)*ca.vertcat(
                ca.horzcat(ca.sin(theta+self.a1),  ca.sin(theta+self.a2), -ca.sin(theta+self.a3), -ca.sin(theta+self.a4)),
                ca.horzcat(ca.cos(theta+self.a1),  ca.cos(theta+self.a2), -ca.cos(theta+self.a3), -ca.cos(theta+self.a4)),
                ca.horzcat(1/(2*0.22), -1/(2*0.22), -1/(2*0.22), 1/(2*0.22))
            )
        return J_for

    def inverse_matrix(self, theta, type):
        if type=="numpy":
            J_inv = (1/self.r)*np.array([
                [math.sin(theta+self.a1), math.cos(theta+self.a1), self.R],
                [math.sin(theta+self.a2), math.cos(theta+self.a2), -self.R],
                [-math.sin(theta+self.a3), -math.cos(theta+self.a3), -self.R],
                [-math.sin(theta+self.a4), -math.cos(theta+self.a4), self.R]
            ], dtype=np.float32)
        elif type=="sym":
            J_inv = (1/self.r)*ca.vertcat(
                (ca.sin(theta+self.a1), ca.cos(theta+self.a1), self.R),
                (ca.sin(theta+self.a2), ca.cos(theta+self.a2), -self.R),
                (-ca.sin(theta+self.a3), -ca.cos(theta+self.a3), -self.R),
                (-ca.sin(theta+self.a4), -ca.cos(theta+self.a4), self.R)
            )

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
        ], dtype=np.float32)
        return rot

    def forward_kinematic(self, v1, v2, v3, v4, angle, type=None):

        if type=="sym":
            vec_for = self.rotation_matrix(angle, type).T@self.forward_matrix(angle, type)@ca.vertcat(v1, v2, v3, v4)
        elif type=="numpy":
            vec_for = self.rotation_matrix(angle, type).T@self.forward_matrix(angle, type)@np.array([v1, v2, v3, v4])

        return vec_for

    def inverse_kinematic(self, angle, vx, vy, vth, type):
        if type=="sym":
            vec_inv = self.inverse_matrix(angle, type)@self.rotation_matrix(angle, type)@ca.vertcat(vx, vy, vth)
        elif type=="numpy":
            vec_inv = self.inverse_matrix(angle, type)@self.rotation_matrix(angle, type)@np.array([vx, vy, vth])
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


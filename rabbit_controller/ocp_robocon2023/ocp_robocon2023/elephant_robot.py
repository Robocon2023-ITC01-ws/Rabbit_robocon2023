import numpy as np
import casadi as ca
import math

class ElephantModel():

    def __init__(self):

        # Kinematic Configuration
        self.al1 = math.pi/4      # radian
        self.al2 = 3*math.pi/4    # radian
        self.al3 = 5*math.pi/4    # radian
        self.al4 = 7*math.pi/4    # radian
        self.L  = 0.285           # [m]
        self.wheel_radius = 0.06 # [m]

    def forward_matrix(self, theta, type=None):
        """
            This function is used to return the matrix of the forward kinematic of
            Omni-directional 4 wheels robot. It will return as Casadi symbolics if
            `type=sym` elif  `type=numpy` it will return as numpy array.
            Reference: Motion Improvement of Four-Wheeled Omnidirectional Mobile Robots for
            Indoor Terrain by Amornphun Phunopas and Shinichi Inoue
        """
        if type=="sym":
            J_for  = (self.wheel_radius/2) * ca.vertcat(
                ca.horzcat(-ca.sin(theta+self.al1), -ca.sin(theta+self.al2), -ca.sin(theta+self.al3), -ca.sin(theta+self.al4)),
                ca.horzcat(ca.cos(theta+self.al1),  ca.cos(theta+self.al2),  ca.cos(theta+self.al3),  ca.cos(theta+self.al4)),
                ca.horzcat(1/(2*self.L), 1/(2*self.L), 1/(2*self.L), 1/(2*self.L))
            )
        elif type=="numpy":
            J_for  = (self.wheel_radius/2) * np.array([
                [-math.sin(theta+self.al1), -math.sin(theta+self.al2), -math.sin(theta+self.al3), -math.sin(theta+self.al4)],
                [ math.cos(theta+self.al1),  math.cos(theta+self.al2),  math.cos(theta+self.al3),  math.cos(theta+self.al4)],
                [1/(self.L), 1/(self.L), 1/(self.L), 1/(self.L)]
            ])
        return J_for

    def inverse_matrix(self, theta, type=None):
        """
            This function is used to return the matrix of the inverse kinematic of
            Omni-directional 4 wheels robot. It will return as Casadi symbolics if
            `type=sym` elif  `type=numpy` it will return as numpy array.
            Reference: Motion Improvement of Four-Wheeled Omnidirectional Mobile Robots for
            Indoor Terrain by Amornphun Phunopas and Shinichi Inoue
        """
        if type=="sym":
            J_inv = (1/self.wheel_radius) * ca.DM([
            [-ca.sin(theta+self.al1), ca.cos(theta+self.al1), 1/self.L],
            [-ca.sin(theta+self.al2), ca.cos(theta+self.al2), 1/self.L],
            [-ca.sin(theta+self.al3), ca.cos(theta+self.al3), 1/self.L],
            [-ca.sin(theta+self.al4), ca.cos(theta+self.al4), 1/self.L],
        ])
        elif type=="numpy":
            J_inv = (1/self.wheel_radius) * np.array([
            [-math.sin(theta+self.al1), math.cos(theta+self.al1), 1/self.L],
            [-math.sin(theta+self.al2), math.cos(theta+self.al2), 1/self.L],
            [-math.sin(theta+self.al3), math.cos(theta+self.al3), 1/self.L],
            [-math.sin(theta+self.al4), math.cos(theta+self.al4), 1/self.L]
        ])
        return J_inv

    def rotation_matrix(self, angle):
        """
            This function is used to return the rotation matrix
            in the cartesian reference. This will return
            the matrix as numpy array.
        """
        rot = np.array([
            [ math.cos(angle),  math.sin(angle),   0],
            [-math.sin(angle),  math.cos(angle),   0],
            [0             ,              0,       1]
        ])
        return rot

    def rotation_matrix_ca(self, angle):
        """
            This function is used to return the rotation matrix
            in the cartesian reference. This will return
            the matrix as casadi symbolics array.
        """
        rot = ca.vertcat(
            ca.horzcat(ca.cos(angle), ca.sin(angle), 0),
            ca.horzcat(-ca.sin(angle), ca.cos(angle), 0),
            ca.horzcat(0, 0, 1)
        )
        return rot

    def forward_kinematic(self, v1, v2, v3, v4, theta, type=None):
        """
            This function is used to return of the forward kinematic of
            Omni-directional 4 wheels robot. It will return as Casadi symbolics if
            `type=sym` elif  `type=numpy` it will return as numpy array.
            Reference: Motion Improvement of Four-Wheeled Omnidirectional Mobile Robots for
            Indoor Terrain by Amornphun Phunopas and Shinichi Inoue
        """
        if type=="sym":
            vec_vel = self.rotation_matrix_ca(theta).T@self.forward_matrix(theta, type)@ca.vertcat(v1, v2, v3, v4)
            # v_x = -(v1*ca.sin(theta+self.al1)+v2*ca.sin(theta+self.al2)+v3*ca.sin(theta+self.al3)+v4*ca.sin(theta+self.al4)) * (self.wheel_radius/2)
            # v_y =  (v1*ca.cos(theta+self.al1)+v2*ca.cos(theta+self.al2)+v3*ca.cos(theta+self.al3)+v4*ca.cos(theta+self.al4)) * (self.wheel_radius/2)
            # v_th=  (v1+v2+v4+v4)/(1/(2*self.wheel_radius))
            # vec_vel = ca.vertcat(v_x, v_y, v_th)
        elif type=="numpy":
            vec_vel = self.rotation_matrix(theta).T@self.forward_matrix(theta, type)@np.array([v1, v2, v3, v4])
        return vec_vel

    def inverse_kinematic(self, vx, vy, vth, theta, type=None):
        """
            This function is used to return of the inverse kinematic of
            Omni-directional 4 wheels robot. It will return as Casadi symbolics if
            `type=sym` elif  `type=numpy` it will return as numpy array.
            Reference: Motion Improvement of Four-Wheeled Omnidirectional Mobile Robots for
            Indoor Terrain by Amornphun Phunopas and Shinichi Inoue.
        """
        if type=="sym":
            vec_inv = self.inverse_matrix(theta, type)@np.array([vx, vy, vth])
        elif type=="numpy":
            vec_inv = self.inverse_matrix(theta, type)@np.array([vx, vy, vth])
        # v1 = vec_inv[0]
        # v2 = vec_inv[1]
        # v3 = vec_inv[2]
        # v4 = vec_inv[3]
        return vec_inv

    def velocity_from_discrete_points(self, k, dt, x_ref, y_ref, theta_ref):
        """
            This function is used to derive the velocity of the robot from the
            given list of number.
        """
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

    def dynamic_model(self, theta,
        vx, vy, vth,
        v1, v2, v3, v4,
        a1, a2, a3, a4,
        dyna_type=None
    ):
        if dyna_type=="derive":
            accel_x_1 = -(self.wheel_radius/2)*(ca.cos(self.al1+theta)*v1+ca.cos(self.al2+theta)*v2+ca.cos(self.al3+theta)*v3+ca.cos(self.al4+theta)*v4)*vth
            accel_x_2 = -(self.wheel_radius/2)*(ca.sin(self.al1+theta)*a1+ca.sin(self.al2+theta)*a2+ca.sin(self.al3+theta)*a3+ca.sin(self.al4+theta)*a4)
            accel_y_1 = -(self.wheel_radius/2)*(ca.sin(self.al1+theta)*v1+ca.sin(self.al2+theta)*v2+ca.sin(self.al3+theta)*v3+ca.sin(self.al4+theta)*v4)*vth
            accel_y_2 =  (self.wheel_radius/2)*(ca.cos(self.al1+theta)*a1+ca.cos(self.al2+theta)*a2+ca.cos(self.al3+theta)*a3+ca.sin(self.al4+theta)*a4)
            accel_x = accel_x_1 + accel_x_2
            accel_y = accel_y_1 + accel_y_2
            accel_th = (self.wheel_radius/(4*self.L))*(a1+a2+a3+a4)
            accel_for = ca.vertcat(accel_x, accel_y, accel_th)
        elif dyna_type=="lagrange":
            pass

        return accel_for

    def forward_dynamic(self):
        pass

    def inverse_dynamic(self):
        pass
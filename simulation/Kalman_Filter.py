import numpy as np
from Quaternions import quat_mult, axis_angle_to_quaternion, quat_conjugate, hemi

class Multiplicative_Extended_Kalman_Filter():
    def __init__(self, P_star_tracker_0, sigma_star, P_b0, sigma_gyro, sigma_bias):
        # Computationally more efficient to define these once rather than create them multiple times each iteration
        self.I3 = np.eye(3) # 3x3 unit matrix
        self.Z3 = np.zeros((3,3)) # 3x3 zeros matrix
        self.I6 = np.eye(6) # 6x6 unit matrix.
        
        # save values for variable Q matrix definition
        self.sigma_gyro = sigma_gyro 
        self.sigma_bias = sigma_bias
        
        self.q = None # estimated quaternion. Must be updated to initial measured quaternion in flight software
        self.last_omega = None # last gyro measurement to perform prediction using body rates of last update step omega_{k-1}. Must be updated to initial measured rate in flight software
        self.b = np.zeros(3) # estimated gyro bias (3x1)
        
        P_theta = P_star_tracker_0 * self.I3
        P_omega = P_b0 * self.I3
        self.P = np.block([[P_theta, self.Z3], # P: 6x6 covariance matrix
                           [self.Z3, P_omega]])
        
        self.R = sigma_star**2 * self.I3 # R: measurement noise covariance [rad]
        # self.H = 0.1*np.eye(3, 6) # H: matrix (Jacobian of measurement model)
        self.H = 0.3*np.eye(3, 6) # H: matrix (Jacobian of measurement model)
        # self.H = np.eye(3, 6) # H: matrix (Jacobian of measurement model)
        
        self.last_time = 0 # time of last prediction step execution (needs to be reinitialized to current time whenever state estimation is switched from inactive to active)
        self.prev_event_used_IMU = True # track whether IMU data was part of last step for decision on Zero-Order_Hold (ZOH) or Midpoint Rule usage
        
        self.last_star_tracker_measurement = 0
        
    def update(self, current_time, omega = None, q_measured=None): # update Kalman filter and return output (event based function)
        dt = current_time - self.last_time
        
        if (omega is not None): # improve accuracy of prediction if new omega is available in order to form midpoint rate
            if self.prev_event_used_IMU:
                self.prediction(dt, 0.5*(self.last_omega + omega)) # predict state using midpoint rule if last prediction didn't use ZOH
            else:
                self.prediction(dt, self.last_omega) # predict state using ZOH
            self.last_omega = omega # if gyro sensor measurement exists, we update the saved gyro values for the next step
            self.prev_event_used_IMU = True
        else:
            self.prediction(dt, self.last_omega) # predict state using ZOH
            self.prev_event_used_IMU = False
            
        if (q_measured is not None): # if star tracker sensor measurement exists, perform correction step as well
            self.correction(q_measured)
        
        self.last_time = current_time # update last time a prediction was performed
        return self.q, (self.last_omega-self.b) # return filtered attitude estimate and bias-correct rate measurement
    
    def prediction(self, dt, omega): # predict next state based on IMU input. Also known as the propagation or estimation step.
        omega = omega - self.b # correct omega with estimated gyro bias
        phi = self.phi_matrix(dt, omega)
        
        # propagate estimated quaternion state based on body rates using the exponential map integration of the quaternion kinematic equation
        theta = np.linalg.norm(omega)*dt
        half = 0.5*theta
        if theta > 1e-8:
            s = np.sin(half)/theta
        else:  # avoid precision loss using series for small angles
            s = 0.5 - theta**2/48.0
        c = np.cos(half)
        delta_q = np.r_[s*omega*dt, c]
        delta_q /= np.linalg.norm(delta_q)
        # self.q = hemi(quat_mult(delta_q, self.q))
        self.q = quat_mult(delta_q, self.q)
        
        Q = self.Q_matrix(dt)
        self.P = phi @ self.P @ phi.T + Q # update covariance matrix
        
    def correction(self, q_measured): # correct/update filter based on measurement input from star tracker. Also known as the innovation or update step.
        # q = hemi(quat_mult(q_measured, quat_conjugate(self.q))) # calculate the innovation quaternion (measurement residual)
        q = quat_mult(q_measured, quat_conjugate(self.q))
        y = 2 * q[:3] # small-angle innovation vector
        
        # LOGARITHM MAP (NO IMPROVEMENT)
        # v = q[:3] # quaternion vector component
        # vn = np.linalg.norm(v) # norm of quaternion vector component
        # s = q[3] # scalar component of quaternion
        # if vn < 1e-12: # small angle protection
        #     y = 2*v
        # else:
        #     rotation_angle = 2*np.atan2(vn, s)
        #     y = rotation_angle/vn*v
        
        K = self.P @ self.H.T @ np.linalg.inv(self.H @ self.P @ self.H.T + self.R) # define Kalman gain
        dx = K @ y
        d_theta = dx[:3]
        db = dx[3:]
        
        # update quaternion
        theta = np.linalg.norm(d_theta)
        if theta < 1e-12:
            delta_q = np.array([0.0, 0.0, 0.0, 1.0])
        else:
            axis = d_theta/theta
            delta_q = axis_angle_to_quaternion(axis, theta)  # scalar-last
        self.q = hemi(quat_mult(delta_q, self.q))

        self.b += db # update gyro bias (bias correction)
        
        # covariance Joseph update
        I6H = (self.I6 - K @ self.H)
        self.P = I6H @ self.P @ I6H.T + K @ self.R @ K.T # update covariance matrix
        self.P = 0.5*(self.P + self.P.T) # enforce symmetry after the Joseph update to kill numerical skew
        
        # MEKF reset mapping (new error coordinates)
        # Gamma ~ I - 0.5*[d_theta]X <- skew matrix built from d_theta
        theta_skew = self.skew(d_theta)
        Gamma = self.I3 - 0.5*theta_skew
        G = np.block([[Gamma, self.Z3],
                      [self.Z3, self.I3]])
        self.P = G @ self.P @ G.T
        
    def phi_matrix(self, dt, omega):
        skew_matrix = self.skew(omega)
        S2 = skew_matrix @ skew_matrix
        norm = np.linalg.norm(omega)
        
        if norm < 1e-6:
            phi11 = self.I3 - skew_matrix*dt + 0.5*S2*dt**2
            phi12 = -self.I3*dt + 0.5*skew_matrix*dt**2 - (1.0/6.0)*S2*dt**3
        else:
            w1, w2, w3 = norm, norm**2, norm**3
            phi11 = self.I3 - skew_matrix * np.sin(w1*dt)/w1 + S2 * (1-np.cos(w1*dt))/w2
            phi12 = skew_matrix *(1-np.cos(w1*dt))/w2 - self.I3*dt - S2 * (w1*dt - np.sin(w1*dt))/w3
        
        phi = np.block([[phi11, phi12],
                        [self.Z3, self.I3]])
        return phi
    
    def Q_matrix(self, dt):
        Q11 = (self.sigma_gyro**2 * dt + (self.sigma_bias**2) * dt**3 / 3.0) * self.I3
        Q12 = (-(self.sigma_bias**2 * dt**2 / 2.0)) * self.I3
        Q22 = (self.sigma_bias**2 * dt) * self.I3
        return np.block([[Q11, Q12],
                         [Q12, Q22]])   
        
    def skew(self, omega):
        """
        Return the 3x3 skew-symmetric matrix (cross-product matrix)
        of a 3-element angular velocity vector omega.
        """
        wx, wy, wz = omega
        return np.array([
            [0,   -wz,  wy],
            [wz,   0,  -wx],
            [-wy, wx,   0]
        ])

if __name__ == "__main__":
    omega = [1,2,3]
    # ekf = Extended_Kalman_Filter()
    print(np.zeros((3,3)))
    print(0.014 * 2*np.pi/360)
    print(np.eye(3,6))
import numpy as np
from Quaternions import quat_mult, axis_angle_to_quaternion, quat_conjugate, hemi

class Extended_Kalman_Filter():
    def __init__(self, dt, P_att0, P_b0, sigma_gyro, sigma_bias, sigma_star): #star_tracker_covariance, omega_covariance, quat_process_noise, IMU_process_noise, sigma_star_tracker):
        I3 = np.eye(3) # 3x3 unit matrix
        Z3 = np.zeros((3,3)) # 3x3 zeros matrix
        
        self.dt = dt # time between sensor updates (IMU in this case) for prediction step
        self.q = None # estimated quaternion. Must be updated to initial measured quaternion in flight software
        self.b = np.zeros(3) # estimated gyro bias (3x1)
        
        P_theta = P_att0 * I3
        P_omega = P_b0 * I3
        self.P = np.block([[P_theta, Z3], # P: 6x6 covariance matrix
                           [Z3, P_omega]])
        
        Q11 = (sigma_gyro**2 * dt + sigma_bias**2 * dt**3/3) * I3
        Q12 = -(sigma_bias**2 * dt**2) * I3
        Q22 = (sigma_bias**2 * dt) * I3
        self.Q = np.block([[Q11, Q12],
                           [Q12, Q22]])        
        self.R = sigma_star**2 * I3 # R: measurement noise covariance [rad]
        self.H = np.eye(3, 6) # H: matrix (Jacobian of measurement model)
        
    def update(self, omega, q_measured=None): # update Kalman filter and return output
        self.prediction(omega) # predict state
        if (q_measured is not None): # if star tracker sensor measurement exists, erform correction step as well
            self.correction(q_measured) # correct state
        return self.q
    
    def prediction(self, omega): # predict next state based on IMU input. Also known as the propogation step.
        omega = omega - self.b # correct oemga with estimated gyro bias
        phi = phi_matrix(self.dt, omega)
        
        # propagate estimated quaternion state based on body rates
        omega_norm = np.linalg.norm(omega)
        if omega_norm < 1e-12:
            pass # do nothing, as we assume no angle change
        else:
            theta = omega_norm * self.dt
            axis = omega / omega_norm
            delta_q = axis_angle_to_quaternion(axis, theta)
            self.q = hemi(quat_mult(delta_q, self.q))
        
        self.P = phi @ self.P @ phi.T + self.Q # update covariance matrix
        
    def correction(self, q_measured): # correct/update filter based on measurement input from star tracker. Also known as the innovation step.
        q = hemi(quat_mult(q_measured, quat_conjugate(self.q))) # calculate the innovation quaternion (measurement residual)
        y = 2 * q[:3] # small-angle innovation vector
        
        K = self.P @ self.H.T @ np.linalg.inv(self.H @ self.P @ self.H.T + self.R) # define Kalman gain
        dx = K @ y
        d_theta = dx[:3]
        db = dx[3:]
        
        # update quaternion
        theta = np.linalg.norm(d_theta)
        axis = d_theta/theta
        delta_q = axis_angle_to_quaternion(axis, theta)
        self.q = hemi(quat_mult(delta_q, self.q))
        
        self.b += db # udpate gyro bias
        I6 = np.eye(6)
        self.P = (I6 - K @ self.H) @ self.P @ (I6 - K @ self.H).T + K @ self.R @ K.T # update covariance matrix

def phi_matrix(dt, omega):
    skew_matrix = skew(omega)
    S2 = skew_matrix @ skew_matrix
    I3 = np.eye(3) # 3x3 unit matrix
    Z3 = np.zeros(3,3) # 3x3 zeros matrix
    norm = np.linalg.norm(omega)
    w1, w2, w3 = norm, norm**2, norm**3
    
    phi11 = I3 - skew_matrix * np.sin(w1*dt)/w1 + S2 * (1-np.cos(w1*dt))/w2
    phi12 = skew_matrix *(1-np.cos(w1*dt))/w2 - I3*dt - S2 * (w1*dt - np.sin(w1*dt))/w3
    
    phi = np.block([[phi11, phi12],
                    [Z3, I3]])
    return phi

def skew(omega):
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
    print(np.zeros(3,3))

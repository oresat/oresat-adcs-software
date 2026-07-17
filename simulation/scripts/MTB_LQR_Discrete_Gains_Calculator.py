"""
This script creates a gain matrix for a Magnetorquer controlled system from a 
modified LQR state-space system designed for systems where the A matrix does
not have full rank.
"""
import numpy as np
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete 

def get_MTB_gain_matrix(J, timestep, max_error, max_rate, orbital_period):
    #----------------- LQR matrices--------------------------------------------
    max_error = max_error # q_vec error
    max_velocity = max_rate # ω_sat
    max_input = 0.04 # max torque (N·m)
    
    Q = np.diag([1/max_error**2, 1/max_error**2, 1/max_error**2, 1/max_velocity**2, 1/max_velocity**2, 1/max_velocity**2])
    R = np.diag([1/max_input**2, 1/max_input**2, 1/max_input**2])
    
    Jxx = J[0,0]
    Jyy = J[1,1]
    Jzz = J[2,2]
    
    A = 0.5*np.eye(6, 6, 3) # A matrix as defined by Miyata & van der Ha and converted to quaternion convention with small approximation
    omega_orbital = 2*np.pi/orbital_period
    
    
    # A[0,1] = omega_orbital
    # A[1,0] = -omega_orbital
    # A[4,3] = (Jyy-Jzz)*omega_orbital/Jxx
    # A[3,4] = (Jzz-Jxx)*omega_orbital/Jyy

    B = np.block([[np.zeros((3,3))], [np.linalg.inv(J)]])
    # B = np.block([[np.zeros((3,3))], [np.zeros((3,3))]])
    # B[3,0] = 1/Jxx
    # B[4,1] = 1/Jyy
    # B[5,2] = 1/Jzz

    C = np.identity(6) # sensors for all inputs
    D = np.zeros((C.shape[0], B.shape[1]))
    
    Ad, Bd, Cd, Dd, dt = cont2discrete((A, B, C, D), timestep)
    P = solve_discrete_are(Ad, Bd, Q, R)
    # K = np.linalg.inv(R+Bd.T @ P @ Bd) @ Bd.T @ P @ Ad
    K = np.linalg.inv(R) @ Bd.T @ P


    A_cl = Ad - Bd @ K  # Discrete closed-loop matrix
    eigvals = np.linalg.eigvals(A_cl)
    
    for i, eig in enumerate(eigvals):
        # print(f"Eigenvalue {i}: {eig}  | Magnitude: {abs(eig)}")
        if abs(eig) > 1:
            print("WARNING: EIGENVALUE OUTSIDE OF UNIT CIRCLE")
    
    return K

if __name__ == "__main__":
    Jxx = 0.01650237
    Jxy = 0.00000711
    Jxz = 0.00004547
    Jyx = 7.115e-6
    Jyy = 0.015962
    Jyz = 0.00003107
    Jzx = 0.00004547
    Jzy = 0.00003107
    Jzz = 0.00651814
    
    J = np.array([[Jxx, Jxy, Jxz], # satellite inertia matrix
                  [Jyx, Jyy, Jyz], 
                  [Jzx, Jzy, Jzz]])
    
    orbital_period = 6000 # seconds
    K = get_MTB_gain_matrix(J, 0.1, 0.1, 0.05, orbital_period)
    print(K)
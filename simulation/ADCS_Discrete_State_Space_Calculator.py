import numpy as np
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete 

def add_integrators(A, B, C):
    """
    Create the augmented A matrix for LQR design with an integrator.
    
    Parameters:
    A (numpy.ndarray): Original system matrix (n x n)
    C (numpy.ndarray): Output matrix (m x n)
    
    Returns:
    numpy.ndarray: Augmented A matrix [(n+m) x (n+m)]
    """
    # Get dimensions
    n = A.shape[0]  # Number of states
    m = C.shape[0]  # Number of outputs
    
    # Verify dimensions
    if A.shape != (n, n):
        raise ValueError("Matrix A must be square")
    if C.shape[1] != n:
        raise ValueError("Number of columns in C must match number of rows in A")
    
    # Create zero matrices
    zero_top_right = np.zeros((n, m))
    zero_bottom_right = np.zeros((m, m))
    
    # Construct augmented A matrix
    A_aug = np.block([
        [A,              zero_top_right],
        [-C,             zero_bottom_right]
    ])
    
    # Construct augmented B matrix (always just added zeros to bottom of vector)
    B_zeros = np.zeros([m,B.shape[1]])
    B_aug = np.block([
        [B],
        [B_zeros]
    ])

    return A_aug, B_aug

def get_gain_matrix(J, Ts, max_error, max_rate, use_integrator = False):
    #Moments of Inertia (kg m^2)
    
    #----------------- LQR matrices--------------------------------------------
    max_error = max_error # q_vec error, previously used 0.05 or .2
    max_velocity = max_rate # ω_sat, previously used 0.02 or .075
    max_integrator = 0.1 # integrator term in Q matrix, integrator state, accumulated error (shouldnt exceed Q values for quaternion error)
    max_input = 0.04 # max torque (N·m) previously used 0.5
    
    Q = np.diag([1/max_error**2, 1/max_error**2, 1/max_error**2, 1/max_velocity**2, 1/max_velocity**2, 1/max_velocity**2, 1/max_integrator**2, 1/max_integrator**2, 1/max_integrator**2])
    R = np.diag([1/max_input**2, 1/max_input**2, 1/max_input**2])
    #--------------------------------------------------------------------------
    
    A = 0.5*np.eye(6, 6, 3) # A matrix: maps ω into q_dot, ω_dot is driven by control input (J^{-1} u)
    B = np.block([[np.zeros((3,3))], [np.linalg.inv(J)]])
    C = np.identity(6) # sensors for all inputs
    C_aug = np.eye(3, 6) # integrator only cares about attitude error, only integrate quaternion values (top half of C matrix)
    
    
    if use_integrator:
        A_aug, B_aug = add_integrators(A, B, C_aug)
        D_aug = np.zeros((C_aug.shape[0], B_aug.shape[1]))

        
        Ad, Bd, Cd, Dd, dt = cont2discrete((A_aug, B_aug, C_aug, D_aug), Ts)
        P = solve_discrete_are(Ad, Bd, Q, R)
        K = np.linalg.inv(R+Bd.T @ P @ Bd) @ Bd.T @ P @ Ad
        
    else:
        D = np.zeros((C.shape[0], B.shape[1]))
        
        Ad, Bd, Cd, Dd, dt = cont2discrete((A, B, C, D), Ts)
        P = solve_discrete_are(Ad, Bd, Q[:6, :6], R)
        K = np.linalg.inv(R+Bd.T @ P @ Bd) @ Bd.T @ P @ Ad
    
    A_cl = Ad - Bd @ K  # Discrete closed-loop matrix
    eigvals = np.linalg.eigvals(A_cl)
    
    for i, eig in enumerate(eigvals):
        print(f"Eigenvalue {i}: {eig}  | Magnitude: {abs(eig)}")
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
    
    useInt = False
    K = get_gain_matrix(J, 0.1, 0.1, 0.05, useInt)
    if useInt:
        print("LQR gain matrix K_int:", K)
    else:
        print("LQR gain matrix K:", K)
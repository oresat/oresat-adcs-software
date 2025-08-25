import numpy as np
import control as ct
from ADCS_Discrete_State_Space_Calculator import get_gain_matrix

def axis_margins(J, K, delay_s=None, axis_names=("x","y","z")):
    """
    J: 3x3 inertia matrix (kg·m^2)
    K: 3x6 LQR gain, u = -K @ [qv; omega]
    delay_s: optional pure delay (s) to model sensing/compute/ZOH (~Ts/2 is a common first guess)
    """
    Jinv = np.linalg.inv(J)
    Kq   = K[:, :3]
    Kw   = K[:, 3:]

    results = {}
    for i, ax in enumerate(axis_names):
        # "Diagonal" PD approximation on axis i
        Kq_i = Kq[i, i]
        Kw_i = Kw[i, i]
        Jinv_ii = Jinv[i, i]

        # Open-loop L_i(s) = ((Kq_i + 2 Kw_i s) * (0.5*Jinv_ii)) / s^2
        num = [Jinv_ii * Kw_i, 0.5 * Jinv_ii * Kq_i]  # s^1, s^0
        den = [1.0, 0.0, 0.0]                         # s^2
        
        L = ct.tf(num, den)

        # Optional delay (Padé approx)
        if delay_s and delay_s > 0:
            num_d, den_d = ct.pade(delay_s, 4)  # 4th-order Padé is a decent start
            D = ct.tf(num_d, den_d)
            L = L * D

        # Margins
        gm, pm, wgc, wpc = ct.margin(L)  # gm as absolute ratio; pm in deg; wgc/wpc rad/s
        gmdB = 20*np.log10(gm) if gm > 0 else np.inf

        results[ax] = dict(
            Kq_i=Kq_i, Kw_i=Kw_i, Jinv_ii=Jinv_ii,
            gm_ratio=gm, gm_dB=gmdB, pm_deg=pm, wgc_rad_s=wgc, wpc_rad_s=wpc
        )
    return results

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
    K = get_gain_matrix(J, 0.1, .1, 0.01, useInt)
    
    bode_dict = axis_margins(J, K)
    print(bode_dict)

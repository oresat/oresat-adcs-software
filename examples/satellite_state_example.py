import numpy as np
from oresat_adcs.classes import dynamics, jday

if __name__ == "__main__":

    # Init a dynamical model without a satellite (uses default satellite)
    x_0   = np.array([5.581498e6, -3.881737e6, 1.421855e4])
    v_0   = np.array([2.708896e3, 3.914674e3, 5.994012e3])
    q_0   = np.array([1, 0,0 ,0])
    w_0   = np.array([0.08726646, 0.08726646, 0.08726646]) # 5 degrees/s / axis. worst case
    whl_0 = np.array([300, -300, -300, 300]) * 0
    t_0 = (2024 ,7, 7, 14, 0, 0)
    dt    = 0.05 # perhaps we want to choose this upstream?

    my_jclock = jday.JClock(*t_0)

    my_state = dynamics.SatelliteState(np.array([x_0, v_0, q_0, w_0, whl_0], dtype=object))
    my_state.attach_clock(my_jclock)
    my_state.update()

    print(my_state.position)
    print(my_state[0])

    print(my_state.GCI_to_ECEF)


import numpy as np
from oresat_adcs.classes import dynamics
from oresat_adcs.classes import jday

if __name__ == "__main__":

    x_0   = np.array([5.581498e6, -3.881737e6, 1.421855e4])
    v_0   = np.array([2.708896e3, 3.914674e3, 5.994012e3])
    q_0   = np.array([1, 0,0 ,0])
    w_0   = np.array([0.08726646, 0.08726646, 0.08726646]) # 5 degrees/s / axis. worst case
    whl_0 = np.array([300, -300, -300, 300]) * 0
    t_0 = (2024 ,7, 7, 14, 0, 0)
    
    
    my_jclock = jday.JClock(*t_0)

    my_state_vector = np.array([x_0, v_0, q_0, w_0, whl_0], dtype=object)

    my_state = dynamics.SatelliteState(my_state_vector)
    # print("\n".join(dir(my_state)))
    
    
    my_state.attach_clock(my_jclock)
    my_state.update_aliases_and_refs()
    # print("--------")
    # print("\n".join(dir(my_state)))
    # print(my_state.GCI_to_ECEF)

    '''
    mod_state = np.array([[1, 0, 0], 
                          [1, 0, 0], 
                          [0, 1, 0, 0], 
                          [0.01, 0.01, 0.01], 
                          [1, 1, 1, 1]], 
                         dtype=object)
    '''
    mod_state = np.array([[1, 0, 0], 
                          [1, 0, 0], 
                          [0, 1, 0, 0]], 
                         dtype=object)
    print(my_state + mod_state)
    pass
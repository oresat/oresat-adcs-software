
import numpy as np
from oresat_adcs.classes import dynamic
from oresat_adcs.configuration import structure

if __name__ == "__main__":


    # Init a dynamical model without a satellite (uses default satellite)
    x_0   = np.array([5.581498e6, -3.881737e6, 1.421855e4])
    v_0   = np.array([2.708896e3, 3.914674e3, 5.994012e3])
    q_0   = np.array([1, 0,0 ,0])
    w_0   = np.array([0.08726646, 0.08726646, 0.08726646]) # 5 degrees/s / axis. worst case
    whl_0 = np.array([300, -300, -300, 300]) * 0
    t_0 = (2024 ,7, 7, 14, 0, 0)
    dt    = 0.05 # perhaps we want to choose this upstream?


    # Create a satellite object
    dimensions = np.array([0.1, 0.1, 0.2])
    max_T = 0.0005 # maximum amout of torque
    torque_limited = True # apply max_T
    products_of_inertia = True # If it is simulated, say true
    my_satellite = structure.Satellite(dimensions, max_T, torque_limited, products_of_inertia)


    # Reduced Model
    # Init a dynamical model without a satellite (uses default satellite)
    default_reduced_model = dynamic.ReducedDynamicalSystem(x_0, v_0, t_0)

    # Init a dynamical model with a satellite object
    custom_reduced_model = dynamic.ReducedDynamicalSystem(x_0, v_0, t_0, satellite=my_satellite)

    # Full Model
    # Init a dynamical model without a satellite (uses default satellite)
    default_model = dynamic.DynamicalSystem(x_0, v_0, q_0, w_0, whl_0, t_0)

    # Init a dynamical model with a satellite object
    custom_model = dynamic.DynamicalSystem(x_0, v_0, q_0, w_0, whl_0, t_0, satellite=my_satellite)

    print("Done")

import json
import numpy as np
from oresat_adcs.classes import jday
from oresat_adcs.configuration import environment, structure, env3, env4

from oresat_adcs.functions import frame

if __name__ == "__main__":
    
    with open("environment_config.json", "r") as config_file:
        environment_config = json.load(config_file)

    my_env = environment.ReducedEnvironment(environment_config)

    my_jday = jday.Clock(2024, 7, 7, 14, 0, 0)
    my_jclock = jday.JClock(2024, 7, 7, 14, 0, 0)

    pos = np.array([0,0,0])
    print(my_env.sun_vector(my_jday, pos))
    pos = np.array([400000,0,0])
    print(my_env.sun_vector(my_jday, pos))
    pos = np.array([0, 400000,0])
    print(my_env.sun_vector(my_jday, pos))
    pos = np.array([0,0,400000])
    print(my_env.sun_vector(my_jday, pos))


    my_jday = jday.Clock(2024, 7, 28, 14, 0, 0)
    
    pos = np.array([0,0,0])
    print(my_env.sun_vector(my_jday, pos))
    pos = np.array([400000,0,0])
    print(my_env.sun_vector(my_jday, pos))
    pos = np.array([0, 400000,0])
    print(my_env.sun_vector(my_jday, pos))
    pos = np.array([0,0,400000])
    print(my_env.sun_vector(my_jday, pos))

    '''
    [-0.27465406  0.88221983  0.38243079]
    [-0.27465653  0.88221919  0.38243051]
    [-0.27465471  0.88221924  0.3824317 ]
    [-0.27465434  0.88222074  0.38242851]
    [-0.58753792  0.74244135  0.32183853]
    [-0.58753967  0.74244018  0.32183802]
    [-0.58753908  0.74244015  0.32183917]
    [-0.58753842  0.74244199  0.32183613]
    '''

    with open("structure_config.json", "r") as config_file:
        structure_config = json.load(config_file)

    my_satellite = structure.Satellite.load("structure_config.json", max_T=0.005, torque_limited=True)

    my_other_env = environment.Environment(my_satellite, hi_fi=False, config=environment_config)



    my_reduced_satellite = structure.ReducedSatellite(mass=3, drag_coefficient=2, dimensions=np.array([0.1, 0.1, 0.2]))
    my_env3 = env3.ReducedEnvironment(my_reduced_satellite, environment_config)

    x_0   = np.array([5.581498e6, -3.881737e6, 1.421855e4])
    v_0   = np.array([2.708896e3, 3.914674e3, 5.994012e3])
    q_0   = np.array([1, 0,0 ,0])
    clock = my_jclock
    GCI_to_ECEF_mat = frame.inertial_to_ecef(clock)
    temp_mag_moment = my_satellite.magnetorquers.actuate(np.array([0, 0, 0]))

    print(my_env3.forces(x_0, v_0))

    my_env4 = env4.Environment(my_satellite, config=environment_config, hi_fi=False)
    
    print(my_env4.env_F_and_T(position=x_0, 
                              velocity=v_0,
                              attitude=q_0,
                              clock=my_jclock,
                              GCI_to_ECEF=GCI_to_ECEF_mat,
                              mag_moment=temp_mag_moment))
    
    pass

import json
import numpy as np
from oresat_adcs.classes import jday
from oresat_adcs.configuration import new_environment, structure
from oresat_adcs.configuration import model

if __name__ == "__main__":
    # position is relative to the center of the earth
    pos = np.array([7e6, 7e6, 7e6])
    vel = np.array([1, 1, 1]) 
    my_jclock = jday.JClock(2024, 7, 7, 14, 0, 0)


    my_orbit = model.OrbitalState(pos, vel, my_jclock)

    print(my_orbit)
    print(*my_orbit)
    my_env = new_environment.OrbitalEnvironment()

    # normally position, velocity, and clock would all change each time
    #my_orbit.update(np.array([1, 1, 1]), vel, my_jclock)
    #print(my_env.SRP_info(my_orbit))

    my_orbit.update(np.array([7e6, 0, 0]), vel, my_jclock)
    print(my_env.SRP_info(my_orbit))
    
    my_orbit.update(np.array([0, 7e6, 0]), vel, my_jclock)
    print(my_env.SRP_info(my_orbit))
    
    my_orbit.update(np.array([0, 0, 7e6]), vel, my_jclock)
    print(my_env.SRP_info(my_orbit))


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

    '''
    with open("structure_config.json", "r") as config_file:
        structure_config = json.load(config_file)

    my_satellite = structure.Satellite.load("structure_config.json", max_T=0.005, torque_limited=True)

    my_other_env = environment.Environment(my_satellite, hi_fi=False, config=environment_config)
    '''

    pass

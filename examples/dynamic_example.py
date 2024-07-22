
import json
import numpy as np
from oresat_adcs.configuration import structure

from oresat_adcs.classes import dynamic

def apply_nparray(keys, dictionary):
    """For every key in keys, turns the value in dictionary from a list to a numpy array"""
    for key in keys:
        dictionary[key] = np.array(dictionary[key])
    return dictionary



def get_object_list(target_class, config_dict):
    """For a certain class, creates a list of instances based on a configuration dictionary"""
    object_list = []
    for name,config in config_dict.items():
        nparray_config = apply_nparray(config["nparrays"], config["keywords"])
        object_list.append(target_class(**nparray_config))

    return object_list




if __name__ == "__main__":
    
    # Set initial conditions
    x_0   = np.array([5.581498e6, -3.881737e6, 1.421855e4])
    v_0   = np.array([2.708896e3, 3.914674e3, 5.994012e3])
    q_0   = np.array([1, 0,0 ,0])
    w_0   = np.array([0.08726646, 0.08726646, 0.08726646]) # 5 degrees/s / axis. worst case
    whl_0 = np.array([300, -300, -300, 300]) * 0
    t_0 = (2024 ,7, 7, 14, 0, 0)
    dt    = 0.05 # perhaps we want to choose this upstream?


    # Make the satellite instance from a configuration file
    with open("structure_config.json", 'r') as structure_config:
        my_config = json.load(structure_config)

    my_instruments = get_object_list(structure.SensitiveInstrument, my_config["instruments"])
    my_torquers = get_object_list(structure.Magnetorquer, my_config["magnetorquers"])
    my_wheels = get_object_list(structure.Wheel, my_config["reaction_wheels"])

    my_mt_system = structure.MagnetorquerSystem(my_torquers)

    max_T = 0.0005,
    torque_limited=True
    my_rw_system = structure.ReactionWheelSystem(my_wheels, max_T, torque_limited)


    base_sat_configs = apply_nparray(my_config["satellite"]["nparrays"], my_config["satellite"]["keywords"])

    my_satellite = structure.Satellite(**base_sat_configs, 
                                       rw_sys=my_rw_system, 
                                       mt_sys=my_mt_system, 
                                       sensitive_instruments=my_instruments)

    # Make the dynamical models
    # Reduced Model
    # Init a dynamical model with a satellite object
    print("\nMaking reduced model with custom satellite")
    custom_reduced_model = dynamic.ReducedDynamicalSystem(x_0, v_0, t_0, my_satellite)

    # Full Model
    # Init a dynamical model with a satellite object
    print("\nMaking full model with custom satellite")
    custom_model = dynamic.DynamicalSystem(x_0, v_0, q_0, w_0, whl_0, t_0, my_satellite)

    print("\nDone")



    pass

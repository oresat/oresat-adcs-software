
import json
import numpy as np
from oresat_adcs.configuration import structure 

from oresat_adcs.configuration import env3, env4
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
    whl_0 = np.array([0, 0, 0, 0])
    cur_cmds = np.array([100, 100, 100])
    whl_cmds = np.array([300, -300, -300, 300])
    t_0 = (2024 ,7, 7, 14, 0, 0)
    dt    = 0.05 # perhaps we want to choose this upstream?


    # Make the satellite instance from a configuration file
    my_reduced_satellite = structure.ReducedSatellite(mass=2.5, drag_coefficient=2, dimensions=np.array([0.1, 0.1, 0.2]))
    my_satellite = structure.Satellite.load("structure_config.json", max_T=0.0005, torque_limited=True)

    print(my_reduced_satellite.cd)


    with open("environment_config.json", "r") as config_file:
        environment_config = json.load(config_file)

    my_reduced_env = env3.ReducedEnvironment(my_reduced_satellite, environment_config)

    my_env = env4.Environment(my_satellite, environment_config, True)
    

    # Make the dynamical models
    # Reduced Model
    # Init a dynamical model with a satellite object
    print("\nMaking reduced model with custom satellite")
    custom_reduced_model = dynamic.ReducedDynamicalSystem(x_0, v_0, t_0, my_reduced_satellite, my_reduced_env)

    print(custom_reduced_model.vector_field(x_0, v_0))
    
    # Full Model
    # Init a dynamical model with a satellite object
    print("\nMaking full model with custom satellite")
    custom_model = dynamic.DynamicalSystem(x_0, v_0, q_0, w_0, whl_0, t_0, my_satellite, my_env)
    print(custom_model.vector_field(x_0, v_0, q_0, w_0, whl_0, cur_cmds, whl_cmds))

    print("\nDone")



    pass


import json
import numpy as np
from oresat_adcs.configuration import structure


def apply_nparray(keys, dictionary):
    for key in keys:
        dictionary[key] = np.array(dictionary[key])
    return dictionary



def get_object_list(target_class, config_dict):
    object_list = []
    for name,config in config_dict.items():
        nparray_config = apply_nparray(config["nparrays"], config["keywords"])
        object_list.append(target_class(**nparray_config))

    return object_list



if __name__ == "__main__":

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

    print(my_satellite)
    pass

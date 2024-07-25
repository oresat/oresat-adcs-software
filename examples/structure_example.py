
import json
import numpy as np
from oresat_adcs.configuration import structure



if __name__ == "__main__":


    my_satellite = structure.Satellite.load("structure_config.json", max_T=0.0005, torque_limited=True)

    print(my_satellite)
    pass

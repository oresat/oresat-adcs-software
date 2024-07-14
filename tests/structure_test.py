
import numpy as np
from oresat_adcs.configuration import structure

if __name__ == "__main__":


    # Create a satellite object

    # dimensions
    # material: wall absorption
    # reaction wheel stuff
    # magnetorquer stuff
    # instruments
    

    # old arguments
    max_T = 0.0005 # maximum amout of torque
    torque_limited = True # apply max_T
    products_of_inertia = True # If it is simulated, say true
    
    # new arguments
    dimensions = np.array([0.1, 0.1, 0.2])

    # Create the object
    my_satellite = structure.Satellite(dimensions=dimensions,
                                       max_T=max_T, 
                                       torque_limited=torque_limited, 
                                       products_of_inertia=products_of_inertia)

    # TEST THE FUNCTIONS

    # multiple v_ref planes should be tested
    print("\nTesting area and center of pressure calcuations")
    v_ref = np.array([1, 0, 0])
    print(my_satellite.area_and_cop(v_ref=v_ref))


    # multiple sun vectors should be used
    print("\nTesting solar radiation pressure and torque")
    solar_radiation_pressure = 0.001
    sun_vector = np.array([1.0, 1.0, 1.0])
    print(my_satellite.srp_forces(SRP=solar_radiation_pressure, S=sun_vector))

    print("\nTesting trag forces")
    print("Drag function not implemented correctly")


    print("\nThe following have dependencies on the structure class and should be tested too:")
    print("\tdynamic.py")


    print("\nDone")

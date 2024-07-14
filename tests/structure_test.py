
import numpy as np
from oresat_adcs.configuration import structure

if __name__ == "__main__":


    # Create a satellite object

    # dimensions
    # material: wall absorption
    # reaction wheel stuff
    # magnetorquer stuff
    # instruments

    """Magnetorquer Test"""
    geometry_type_1 = "Square"
    geometry_type_2 = "Rod"
    geometry_type_3 = "LinearRod"
    # maybe just pass resistance instead of a type?
    # uhhh, get inductance too???

    # z, input is 10,000 for a current of ~0.288700 A
    # x, y, input is 10,000 for a current of ~0.062500 A
    my_mt_x = structure.Magnetorquer("LinearRod", np.array([1.0, 0.0, 0.0]), max_A=0.0625)
    my_mt_y = structure.Magnetorquer("Rod", np.array([0.0, 1.0, 0.0]), max_A = 0.0625)
    my_mt_z = structure.Magnetorquer("Square", np.array([0.0, 0.0, 1.0]), max_A = 0.2887)
    
    print("\n\nTesting MT Functions")
    print("\nactuate functions")
    print(my_mt_x.actuate(1))
    print(my_mt_y.actuate(1))
    print(my_mt_z.actuate(1))

    print("\ngetCurrent functions")
    # pass the desired magnetic moment
    print(my_mt_x.getCurrent(np.array([0.1, 0.1, 0.1])))
    print(my_mt_y.getCurrent(np.array([0.1, 0.1, 0.1])))
    print(my_mt_z.getCurrent(np.array([0.1, 0.1, 0.1])))


    print("\nPower functions")
    print(my_mt_x.power(0.01))
    print(my_mt_y.power(0.01))
    print(my_mt_z.power(0.01))




    """Magnetorquer System Test"""
    print("\n\nMagnetic system test")
    linearized=True
    max_A = 0.3
    my_mt_sys = structure.MagnetorquerSystem(linearized, max_A)

    print("\nDistributing current")
    print(my_mt_sys.distribute(np.array([1, 1, 1])))
    print("\nActuate the magnetorquers")
    print(my_mt_sys.actuate(np.array([1, 1, 1])))
    print("\nPower consumption")
    print(my_mt_sys.power(np.array([1, 1, 1])))


    print("\nModifying Magnetorqer system")
    # Throw in an extra MT just to mess with them, also try just one or two MTs
    my_mt_sys.set_torquers([my_mt_x, my_mt_y, my_mt_z])
    
    #my_mt_other = structure.Magnetorquer("LinearRod", np.array([1.0, 1.0, 1.0]), max_A = 0.2887)
    #my_mt_sys.set_torquers([my_mt_x, my_mt_y, my_mt_z, my_mt_other])

    print("\nDistributing current")
    print(my_mt_sys.distribute(np.array([1, 1, 1])))
    print("\nActuate the magnetorquers")
    print(my_mt_sys.actuate(np.array([1, 1, 1])))
    print("\nPower consumption")
    print(my_mt_sys.power(np.array([1, 1, 1])))




    """Wheel Test"""
    inclination = np.pi / 3    
    azimuth = np.pi / 4
    parallel_moment = 1.64023e-6
    orthogonal_moment = 1.02562e-6
    index = 1

    my_wheel = structure.Wheel(inclination, azimuth, parallel_moment, orthogonal_moment, index)

    # a = torque / parallel moment scalar
    print("\n\nTesting wheel object")
    print(my_wheel.acceleration(1))
    print(my_wheel.momentum(1))
    print(my_wheel.torque(1))






    """Wheel System Test"""
    # Currently hardcoded to use four
    max_T = 0.0005
    torque_limited = True

    my_rw_system = structure.ReactionWheelSystem(inclination, azimuth, parallel_moment, orthogonal_moment, max_T, torque_limited)

    print("\n\nTesting reaction wheel system")
    command_torque = 1.0
    velocities = np.array([1, 1, 1, 1])
    accelerations = np.array([1, 1, 1, 1])
    print("\nAccelerations")
    print(my_rw_system.accelerations(command_torque))
    print("\nMomentum")
    print(my_rw_system.momentum(velocities))
    print("\nTorque")
    print(my_rw_system.torque(accelerations))











    """Satellite Tests"""
    # old arguments
    print("\n\nTesting satellite")
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

    my_satellite = structure.Satellite(dimensions=dimensions,
                                       max_T=max_T, 
                                       torque_limited=torque_limited, 
                                       products_of_inertia=products_of_inertia,
                                       reaction_wheel_system=my_rw_system)

    

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

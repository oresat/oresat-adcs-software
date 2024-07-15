
import numpy as np
from oresat_adcs.configuration import structure








if __name__ == "__main__":

    """Sensitive Instrument Test"""

    my_instruments= [structure.SensitiveInstrument(np.array([0, 0, -1]), bounds=[15, 100], forbidden=[True, False], obj_ids=[0]),
                     structure.SensitiveInstrument(np.array([0, -1, 0]), bounds=[180, 180], forbidden=[False, False], obj_ids=[])]



    """Magnetorquer Test"""
    geometry_type_1 = "Square"
    geometry_type_2 = "Rod"
    geometry_type_3 = "LinearRod"
    # maybe just pass resistance instead of a type?
    # uhhh, get inductance too???

    # z, input is 10,000 for a current of ~0.288700 A
    # x, y, input is 10,000 for a current of ~0.062500 A
    my_mt_x = structure.Magnetorquer("LinearRod", np.array([1.0, 0.0, 0.0]), max_A=0.0625)
    my_mt_y = structure.Magnetorquer("LinearRod", np.array([0.0, 1.0, 0.0]), max_A = 0.0625)
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
    # make torqer system
    linearized = True
    mt_type = "LinearRod" if linearized else "Rod"

    torquers = [structure.Magnetorquer(mt_type, np.array([1, 0, 0]), 0.0625),
                structure.Magnetorquer(mt_type, np.array([0, 1, 0]), 0.0625),
                structure.Magnetorquer("Square", np.array([0, 0, 1]), 0.2887)]


    print("\n\nMagnetic system test")
    my_mt_system = structure.MagnetorquerSystem(torquers)

    
    print("\nDistributing current")
    print(my_mt_system.distribute(np.array([1, 0.01, 0.01])))
    print(my_mt_system.distribute(np.array([0.01, 1, 0.01])))
    print(my_mt_system.distribute(np.array([0.01, 0.01, 1])))

    print("\nActuate the magnetorquers")
    print(my_mt_system.actuate(np.array([1, 1, 1])))
    print("\nPower consumption")
    print(my_mt_system.power(np.array([1, 1, 1])))




    """Wheel Test"""
    inclination = np.pi / 3    
    azimuth = np.pi / 4
    parallel_moment = 1.64023e-6
    orthogonal_moment = 1.02562e-6

    given_axes = [np.array([np.sin(inclination)*np.cos(azimuth+rw_index*np.pi/2),
                            np.sin(inclination)*np.sin(azimuth+rw_index*np.pi/2),
                            np.cos(inclination)]) for rw_index in range(4)]


    my_wheel_1 = structure.Wheel(given_axes[0], parallel_moment, orthogonal_moment)
    my_wheel_2 = structure.Wheel(given_axes[1], parallel_moment, orthogonal_moment)
    my_wheel_3 = structure.Wheel(given_axes[2], parallel_moment, orthogonal_moment)
    my_wheel_4 = structure.Wheel(given_axes[3], parallel_moment, orthogonal_moment)

    # a = torque / parallel moment scalar
    print("\n\nTesting wheel object")
    print(my_wheel_1.acceleration(1))
    print(my_wheel_2.momentum(1))
    print(my_wheel_3.torque(1))



    """Wheel System Test"""
    # Currently hardcoded to use four
    max_T = 0.0005
    torque_limited = True

    my_rw_system = structure.ReactionWheelSystem([my_wheel_1, my_wheel_2, my_wheel_3, my_wheel_4], max_T, torque_limited)

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

    print("\nCheck the axis of all default reaction wheels")
    print(my_rw_system.axes)



    """Satellite Tests"""
    

    # old arguments
    print("\n\nTesting satellite")
    # MKS units
    # kilograms
    mass = (21.66*4 + 2419.56) / 1000
    # some unit
    absorption = 0.84
    # dimensionless drag coefficient
    drag = 2
    # principal moments without reaction wheels
    principal_moments = np.array([1.378574142e-2,
                                  1.378854578e-2,
                                  5.49370596e-3])
    # products of inertia: xy, xz, yz
    principal_products = principal_moments*0.1


    products_of_inertia = True # If it is simulated, say true

    # new arguments
    dimensions = np.array([0.1, 0.1, 0.2])

    # Create the object
    
    my_satellite = structure.Satellite(mass=mass,
                                       dimensions=dimensions,
                                       absorption=absorption,
                                       drag_coeff = drag,
                                       principal_moments=principal_moments,
                                       product_moments = principal_products,
                                       products_of_inertia=products_of_inertia,
                                       rw_sys=my_rw_system, 
                                       mt_sys=my_mt_system,
                                       sensitive_instruments=my_instruments)

    

    

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


import json
import numpy as np
from oresat_adcs.classes import sensors
from oresat_adcs.configuration import structure, environment



if __name__ == "__main__":

    my_instruments= [structure.SensitiveInstrument(np.array([0, 0, -1]), bounds=[15, 100], forbidden=[True, False], obj_ids=[0]),
                     structure.SensitiveInstrument(np.array([0, -1, 0]), bounds=[180, 180], forbidden=[False, False], obj_ids=[])]

    # make torqer system
    linearized = True
    mt_type = "LinearRod" if linearized else "Rod"
    torquers = [structure.Magnetorquer(mt_type, np.array([1, 0, 0]), 0.0625),
                structure.Magnetorquer(mt_type, np.array([0, 1, 0]), 0.0625),
                structure.Magnetorquer("Square", np.array([0, 0, 1]), 0.2887)]
    my_mt_system = structure.MagnetorquerSystem(torquers)

    # make reaction wheel system
    inclination = np.pi / 3
    azimuth = np.pi / 4
    parallel_moment = 1.64023e-6
    orthogonal_moment = 1.02562e-6
    max_T = 0.0005
    torque_limited = True
    given_axes = [np.array([np.sin(inclination)*np.cos(azimuth+rw_index*np.pi/2),
                            np.sin(inclination)*np.sin(azimuth+rw_index*np.pi/2),
                            np.cos(inclination)]) for rw_index in range(4)]
    wheels = [structure.Wheel(given_axes[0], parallel_moment, orthogonal_moment),
              structure.Wheel(given_axes[1], parallel_moment, orthogonal_moment),
              structure.Wheel(given_axes[2], parallel_moment, orthogonal_moment),
              structure.Wheel(given_axes[3], parallel_moment, orthogonal_moment)]
    my_rw_system = structure.ReactionWheelSystem(wheels, max_T, torque_limited)


    # MKS units
    mass = (21.66*4 + 2419.56) / 1000
    absorption = 0.84
    drag = 2
    principal_moments = np.array([1.378574142e-2,
                                  1.378854578e-2,
                                  5.49370596e-3])
    # products of inertia: xy, xz, yz
    products_of_inertia = principal_moments*0.1
    dimensions = np.array([0.1, 0.1, 0.2])

    my_env = environment.OrbitalEnvironment(hi_fi=True) 
    sensors = [sensors.GPS_pos(mean=0, std_dev=30, env=my_env),
               sensors.GPS_vel(mean=0, std_dev=2, env=my_env),
               sensors.StarTracker(mean=0, std_dev=0.75e-7, env=my_env, size=4),
               sensors.Gyro(arw_mean=0, arw_std_dev=2.79e-4, 
                                rrw_mean=0, rrw_std_dev=8.73e-7, 
                                init_bias=3.15e-5, env=my_env),
               sensors.Wheel_vel(mean=0, std_dev=0.0001, env=my_env, size=4),
               sensors.Magnetometer(mean=0, std_dev=4e-8, env=my_env), # from datasheet
               sensors.SunSensor(mean=0, std_dev=1e-6, env=my_env)]
    
    my_satellite = structure.Satellite(mass=3.0,
                                        dimensions=np.array([0.1, 0.1, 0.2]),
                                        absorption=0.84,
                                        drag_coeff=1.0,
                                        principal_moments=principal_moments,
                                        product_moments=products_of_inertia,
                                        reduced=False,
                                        sensors=sensors,
                                        rw_sys=my_rw_system,
                                        mt_sys=my_mt_system,
                                        sensitive_instruments=my_instruments)
    
    # double check if it can be loaded from json file
    #my_satellite = structure.Satellite.load("structure_config.json", max_T=0.0005, torque_limited=True)
    #print(my_satellite)
    pass

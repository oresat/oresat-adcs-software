
import json
import numpy as np
from oresat_adcs.configuration import new_environment, structure

from oresat_adcs.classes import jday, sensor

#from oresat_adcs.configuration import structure
from oresat_adcs.configuration import model

if __name__ == "__main__":

    # Init a dynamical model without a satellite (uses default satellite)
    x_0   = np.array([5.581498e6, -3.881737e6, 1.421855e4])
    v_0   = np.array([2.708896e3, 3.914674e3, 5.994012e3])
    q_0   = np.array([1, 0,0 ,0])
    w_0   = np.array([0.08726646, 0.08726646, 0.08726646]) # 5 degrees/s / axis. worst case
    whl_0 = np.array([300, -300, -300, 300]) * 0
    t_0 = (2024 ,7, 7, 14, 0, 0)
    dt    = 0.05 # perhaps we want to choose this upstream?

    my_jclock = jday.JClock(*t_0)

    
    '''Test new environment'''
    my_env = new_environment.OrbitalEnvironment(hi_fi=True)



    '''REDUCED SATELLITE'''
    
    my_reduced_satellite = model.ReducedSatelliteModel(environment=my_env, 
                                                       mass=3.0, 
                                                       drag_coefficient=1.0, 
                                                       area=0.02,
                                                       absorption=0.84,
                                                       position=x_0,
                                                       lin_vel=v_0,
                                                       date_and_time=t_0)


    print(my_reduced_satellite.vector_field(x_0, v_0))


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

    '''
    # The sensor shouldn't have to depend on the model, just pass a value then have the sensor add noise or whatever

    sensors     = [sensor.GPS_pos(mean=0, std_dev=30, model=None),
                            sensor.GPS_vel(mean=0, std_dev=2, model=None),
                            sensor.StarTracker(mean=0, std_dev=0.75e-7, model=None),
                            sensor.Gyro(arw_mean=0, arw_std_dev=2.79e-4, rrw_mean=0, rrw_std_dev=8.73e-7, init_bias=3.15e-5, model=None),
                            sensor.Wheel_vel(mean=0, std_dev=0.0001, model=None),
                            sensor.Magnetometer(mean=0, std_dev=4e-8, model=None), # from datasheet
                            sensor.SunSensor(mean=0, std_dev=1e-6, model=None)
                            ]
    '''
    sensors = []
    my_satellite = model.SatelliteModel(environment=my_env,
                                        mass=3.0,
                                        dimensions=np.array([0.1, 0.1, 0.2]),
                                        absorption=0.84,
                                        drag_coeff=1.0,
                                        principal_moments=principal_moments,
                                        product_moments=products_of_inertia,
                                        reduced=False,
                                        date_and_time=t_0,
                                        sensors=sensors,
                                        rw_sys=my_rw_system,
                                        mt_sys=my_mt_system,
                                        sensitive_instruments=my_instruments)
    
    print(my_satellite.env_F_and_T(position=x_0, velocity=v_0, attitude=q_0, clock=my_jclock, mag_moment=np.array([0, 0, 0])))

    print(my_satellite.vector_field(position=x_0, lin_vel=v_0, attitude=q_0, 
                                    body_ang_vel=np.array([0, 0, 0]), 
                                    wheel_vel=np.array([0, 0, 0, 0],),
                                    cur_cmd=np.array([0, 0, 0]),
                                    whl_accl=np.array([0, 0, 0, 0])))

    pass

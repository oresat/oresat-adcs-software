import numpy as np

from oresat_adcs.classes import jday, dynamics, new_sensors
from oresat_adcs.configuration import environment

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

    my_state = dynamics.SatelliteState(np.array([x_0, v_0, q_0, w_0, whl_0], dtype=object))
    my_state.attach_clock(my_jclock)
    my_state.update()

    my_env = environment.OrbitalEnvironment(hi_fi=True)
   


    gps_pos = new_sensors.GPS_pos(mean=0, std_dev=30, env=my_env)
    gps_vel = new_sensors.GPS_vel(mean=0, std_dev=2, env=my_env)
    star_tracker = new_sensors.StarTracker(mean=0, std_dev=0.75e-7, env=my_env, size=4)
    gyroscope = new_sensors.Gyro(arw_mean=0, arw_std_dev=2.79e-4, 
                                 rrw_mean=0, rrw_std_dev=8.73e-7, 
                                 init_bias=3.15e-5, env=my_env)
    wheel_vel_sensors = new_sensors.Wheel_vel(mean=0, std_dev=0.0001, env=my_env, size=4)
    mag = new_sensors.Magnetometer(mean=0, std_dev=4e-8, env=my_env) # from datasheet
    sun = new_sensors.SunSensor(mean=0, std_dev=1e-6, env=my_env)


    print(my_state.position)
    print(gps_pos.measurement(my_state, noisy=False))
    print(gps_pos.measurement(my_state, noisy=True))
    print("\n")

    print(my_state.velocity)
    print(gps_vel.measurement(my_state, noisy=False))
    print(gps_vel.measurement(my_state, noisy=True))
    print("\n")


    print(my_state.attitude)
    print(star_tracker.measurement(my_state, noisy=False))
    print(star_tracker.measurement(my_state, noisy=True))
    print("\n")
   
    
    print(my_state.body_ang_vel)
    print(gyroscope.measurement(my_state, noisy=False))
    gyroscope.propagate(100)
    print(gyroscope.measurement(my_state, noisy=True))
    gyroscope.propagate(100)
    print(gyroscope.measurement(my_state, noisy=True))
    gyroscope.propagate(100)
    print(gyroscope.measurement(my_state, noisy=True))
    print("\n")
    

    print(my_state.wheel_vel)
    print(wheel_vel_sensors.measurement(my_state, noisy=False))
    print(wheel_vel_sensors.measurement(my_state, noisy=True))
    print("\n")


    print(my_env.magnetic_field(my_state))
    # technically if the quaternion were different, the noise-less 
    print(mag.measurement(my_state, noisy=False))
    print(mag.measurement(my_state, noisy=True))
    print("\n")

    print(my_env.SRP_info(my_state)[2])
    print(sun.measurement(my_state, noisy=False))
    print(sun.measurement(my_state, noisy=True))

import unittest
import numpy as np
from oresat_adcs.classes import jday, dynamics, new_sensors
from oresat_adcs.configuration import environment

class SensorTestMethods(unittest.TestCase):
    x_0   = np.array([5.581498e6, -3.881737e6, 1.421855e4])
    v_0   = np.array([2.708896e3, 3.914674e3, 5.994012e3])
    q_0   = np.array([1, 0, 0 ,0])
    w_0   = np.array([0.08726646, 0.08726646, 0.08726646]) # 5 degrees/s / axis. worst case
    whl_0 = np.array([300, -300, -300, 300]) * 0
    t_0 = (2024 ,7, 7, 14, 0, 0)

    my_jclock = jday.JClock(*t_0)

    my_state = dynamics.SatelliteState(np.array([x_0, v_0, q_0, w_0, whl_0], dtype=object))
    my_state.attach_clock(my_jclock)
    my_state.update()
    
    my_env = environment.OrbitalEnvironment(hi_fi=True)


    def test_gps_pos(self):
        '''Tests gps position''' 
        gps_pos = new_sensors.GPS_pos(mean=0, std_dev=30, env=self.my_env)
        self.assertTrue((self.my_state.position == gps_pos.measurement(self.my_state, noisy=False)).all())
        self.assertFalse((self.my_state.position == gps_pos.measurement(self.my_state, noisy=True)).all())
        
    def test_gps_vel(self):
        '''Tests gps velocity''' 
        gps_vel = new_sensors.GPS_vel(mean=0, std_dev=2, env=self.my_env)
        self.assertTrue((self.my_state.velocity == gps_vel.measurement(self.my_state, noisy=False)).all())
        self.assertFalse((self.my_state.velocity == gps_vel.measurement(self.my_state, noisy=True)).all())

    def test_star_tracker(self):
        '''Tests star tracker''' 
        star_tracker = new_sensors.StarTracker(mean=0, std_dev=0.75e-7, env=self.my_env, size=4)
        self.assertTrue((self.my_state.attitude == star_tracker.measurement(self.my_state, noisy=False)).all())
        self.assertFalse((self.my_state.attitude == star_tracker.measurement(self.my_state, noisy=True)).all())
        
    def test_gyroscope(self):
        '''Tests gyroscope''' 
        gyroscope = new_sensors.Gyro(arw_mean=0, arw_std_dev=2.79e-4, 
                                 rrw_mean=0, rrw_std_dev=8.73e-7, 
                                 init_bias=3.15e-5, env=self.my_env)
        self.assertTrue((self.my_state.body_ang_vel == gyroscope.measurement(self.my_state, noisy=False)).all())
        self.assertFalse((self.my_state.body_ang_vel == gyroscope.measurement(self.my_state, noisy=True)).all())
        
    def test_gyroscope_propogation(self):
        '''Tests gyroscope bias propogation''' 
        gyroscope = new_sensors.Gyro(arw_mean=0, arw_std_dev=2.79e-4, 
                                 rrw_mean=0, rrw_std_dev=8.73e-7, 
                                 init_bias=3.15e-5, env=self.my_env)
        self.assertTrue((self.my_state.body_ang_vel == gyroscope.measurement(self.my_state, noisy=False)).all())
        gyroscope.propagate(0.1)
        self.assertFalse((self.my_state.body_ang_vel == gyroscope.measurement(self.my_state, noisy=True)).all())
        # check if bias changed
        self.assertFalse((gyroscope.bias == np.array([0, 0, 0])).all())

       
    def test_wheel_vel(self):
        '''Tests wheel velocity sensros''' 
        wheel_vel_sensors = new_sensors.Wheel_vel(mean=0, std_dev=0.0001, env=self.my_env, size=4)
        self.assertTrue((self.my_state.wheel_vel == wheel_vel_sensors.measurement(self.my_state, noisy=False)).all())
        self.assertFalse((self.my_state.wheel_vel == wheel_vel_sensors.measurement(self.my_state, noisy=True)).all())
        
     
    def test_magnetometer(self):
        '''Tests magnetometer'''   
        mag = new_sensors.Magnetometer(mean=0, std_dev=4e-8, env=self.my_env) # from datasheet
        # check against quaternion
        self.assertTrue((self.my_env.magnetic_field(self.my_state) == mag.measurement(self.my_state, noisy=False)).all())
        self.assertFalse((self.my_env.magnetic_field(self.my_state) == mag.measurement(self.my_state, noisy=True)).all())


    def test_sun_sensor(self):
        '''Tests sun sensors'''
        sun = new_sensors.SunSensor(mean=0, std_dev=1e-6, env=self.my_env)
        self.assertTrue((self.my_env.SRP_info(self.my_state)[2] == sun.measurement(self.my_state, noisy=False)).all())
        self.assertFalse((self.my_env.SRP_info(self.my_state)[2] == sun.measurement(self.my_state, noisy=True)).all())
        
       


if __name__ == "__main__":
    # Naively assumes that noise will be nonzero
    unittest.main() 
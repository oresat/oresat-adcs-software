import unittest
import numpy as np
from oresat_adcs.classes import jday, dynamics, sensors
from oresat_adcs.configuration import environment

class SensorTestMethods(unittest.TestCase):

    def test_gpspos(self):
        x_0   = np.array([5.581498e6, -3.881737e6, 1.421855e4])
        v_0   = np.array([2.708896e3, 3.914674e3, 5.994012e3])
        q_0   = np.array([1, 0,0 ,0])
        w_0   = np.array([0.08726646, 0.08726646, 0.08726646]) # 5 degrees/s / axis. worst case
        whl_0 = np.array([300, -300, -300, 300]) * 0
        t_0 = (2024 ,7, 7, 14, 0, 0)
        dt    = 0.05

        my_jclock = jday.JClock(*t_0)

        my_state = dynamics.SatelliteState(np.array([x_0, v_0, q_0, w_0, whl_0], dtype=object))
        my_state.attach_clock(my_jclock)
        my_state.update()
        
        my_env = environment.OrbitalEnvironment(hi_fi=True)
        
        
        gps_pos = sensors.GPS_pos(mean=0, std_dev=30, env=my_env)
        self.assertEqual(self.my_state.position, gps_pos.measurement(my_state, noisy=False))
        self.assertNotEqual(self.my_state.position, gps_pos.measurement(my_state, noisy=True))



if __name__ == "__main__":
    unittest.main() 
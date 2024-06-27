
# error test oresat_adcs.generic.frame 
# import datetime

import numpy as np
from oresat_adcs.generic import jday, frame

if __name__ == "__main__":


    print("\n\nTESTING FRAME\n\n")
    test_clock = jday.Clock(year=2024, month=6, day=24, hour=15, minute=12, second=5)

    # inertial to ecef, requires a jday clock, returns transformation matrix
    print("Testing inertial to ecef: \n", frame.inertial_to_ecef(test_clock))
    
    # lvlh to inertial, requires a numpy array for position and velocity, returns transformation matrix
    print("Testing lvlh_to_inertial: \n", frame.lvlh_to_inertial(np.array([1, 2, 3]), np.array([4, 5, 6])))

    # geodetic to ecef, requires latitude, longitude, and altitude, units for h?, returns ecef coordinates
    print("Testing geodetic to ecef: \n", test_ecef:=frame.geodetic_to_ECEF(0, 0, 100))

    # ecef_to_lla, requires ECEF frame, returns llh
    print("Testing ecef to lla: \n", frame.ecef_to_lla(test_ecef))

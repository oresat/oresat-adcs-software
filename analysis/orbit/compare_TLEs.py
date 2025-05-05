
from datetime import datetime, timezone
from sgp4.api import Satrec
import ephem

from oresat_adcs.classes import jday
from oresat_adcs.functions import frame
# Test if C code made it fast
#from sgp4.api import accelerated
#print(accelerated)

if __name__ == "__main__":
    now = datetime.utcnow()
    print(now)
    my_clock = jday.JClock(now.year, now.month, now.day, now.hour, now.minute, now.second)

    TLE_A = ['1 52017U 22026K   24242.00664388  .00309760  00000-0  20745-2 0  9991',
             '2 52017  97.4855 260.4575 0002910 260.2863  99.8072 15.74373615137078']

    to_compare = [TLE_A]
    # USE SGP4
    for tle in to_compare:
        tle
        sat_sgp4 = Satrec.twoline2rv(tle[0], tle[1])

        julian_float = my_clock.julian_date()
        # km, km/s
        e, r, v = sat_sgp4.sgp4(julian_float, 0.0)
        # convert km to m
        r = [thing*1000 for thing in r]
        v = [thing*1000 for thing in v]

        print("position:", r)
        print("velocity:", v)

        # CHECK SGP4 for latitude and longitude
        GCI_to_ECEF = frame.inertial_to_ecef(my_clock)
        r_ecef = GCI_to_ECEF.dot(r)
        lat, long, h = frame.ecef_to_lla(r_ecef)
        print("h:\t", h)



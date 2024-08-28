
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
    # print(now)
    my_clock = jday.JClock(now.year, now.month, now.day, now.hour, now.minute, now.second)


    tle1 = "1 98867U          24238.20000000  .00000000  00000-0  20199-3 0    06"
    tle2 = "2 98867  97.4404 314.0487 0008202 330.6740  42.2938 15.18964090    01"

    # USE SGP4
    sat_sgp4 = Satrec.twoline2rv(tle1, tle2)

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
    print("sgp4 + oresat_adcs:\t", lat, long, h)


    # USE PYEPHEM

    sat_ephem = ephem.readtle("ORESAT0.5 (ORESAT0.5)", tle1, tle2)
    now_string = f"{now.year}/{now.month}/{now.day} {now.hour}:{now.minute}:{now.second}"
    sat_ephem.compute(now_string)
    print("just using pyephem:\t", sat_ephem.sublat, sat_ephem.sublong, sat_ephem.elevation)


# Orbit calculator
import numpy as np
from Basilisk.utilities import orbitalMotion, macros
import json
import preset_utils

def calc_bsk_orbit(config):
    # true orbit parameters for SENTINEL mission
    oe = orbitalMotion.ClassicElements()
    
    # radius of Earth [m]
    Re = 6371e3 
    # standard gravitational parameter of earth [m^3 s^-2]
    mu_earth = 398600436000000.0 

    ra = Re + config["apoapsis"]
    rp = Re + config["periapsis"]

    # semi-major axis [meters] (altitude + earth's radius)
    oe.a = 0.5*(rp + ra)
    print(f"Semi Major Axis [m]: {oe.a}")

    # eccentricity
    oe.e = (ra - rp)/(ra + rp)
    print(f"Eccentricity: {oe.e}")

    # inclination
    oe.i = config["inclination_deg"] * macros.D2R # [degrees]
    print(f"Inclination [rad]: {oe.i}")

    # RAAN or Longitude of Ascending Node
    oe.Omega = config["ascending_node_deg"] * macros.D2R # [degrees]
    print(f"RAAN: {oe.Omega}")

    # argument of periapsis
    oe.omega = config["arg_periapsis_deg"] * macros.D2R   # sets perigee vector angle from ascending node in the orbital plane [degrees]
    print(f"Argument of periapsis: {oe.omega}")

    # true anomaly
    oe.f = config["true_anomaly_deg"] * macros.D2R      # where the satellite is on the ellipse at epoch (start of sim) [degrees]
    print(f"True Anomaly [rad]: {oe.f}")

    rN, vN = orbitalMotion.elem2rv(mu_earth, oe)
    oe = orbitalMotion.rv2elem(mu_earth, rN, vN)  # this stores consistent initial orbit elements, fixes numerical errors, particulary with perfectly circular orbits. Consult ChatGPT for detailed explanation.
    orbital_period = 2*np.pi*np.sqrt(oe.a**3/mu_earth) # define orbital period for plotting
    print(f"orbital period: {orbital_period} sec ({orbital_period / 60.0} min)")

if __name__ == "__main__":
    orbit_config = {
        "apoapsis": 600e3,
        "periapsis": 580e3,
        "inclination_deg": 98.7,
        "ascending_node_deg": 130.0,
        "arg_periapsis_deg": 0.0,
        "true_anomaly_deg": 82.0
    }

    calc_bsk_orbit(orbit_config)

    blah = preset_utils.npdict_to_plaindict(orbit_config)
    with open("example_orbit.json", "w") as fd:
        json.dump(blah, fd, indent=2)


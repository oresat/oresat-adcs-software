import numpy as np
import geometry # can't remember whether these imports make namespace "flat"
import system_definition

# physics
# angular momentum of all wheels together (along their respective spin axes), body ref
def rw_sys_momentum(wheel_vel, body_ang_vel):
    return sum([RW_MOMENT_PARALLEL[i] * (np.dot(axis, body_ang_vel) +
                wheel_vel[i]) * axis
                for i, axis in enumerate(WHEEL_AXES)])

# torque of wheels, body ref, assuming acceleration command instanteous.
# at some point need to either model internal wheel model or compensate somehow
def rw_sys_torque(accl_cmd, body_ang_accl):
    return sum([axis * RW_MOMENT_PARALLEL[i] * (np.dot(axis, body_ang_accl) +
                                                accl_cmd[i])
                for i, axis in enumerate(WHEEL_AXES)])

# exponentially decaying model atmosphere page 406
# h is height above geode in m (i.e., in geodetic coordinates)
def atmo_density(h):
    h /= 1000 # convert height to km
    if 350 <= h and h < 400:
        h_0 = 350 # km
        rho_0 = 9.158 * 10**(-3) # kg/km^3
        H = 56.4 # km
    elif 400 <= h and h < 450:
        h_0 = 400 # km
        rho_0 = 3.727 * 10**(-3) # kg/km^3
        H = 59.4 # km
    else:
        print("height out of bounds!", h) # when less lazy, allow for decaying orbit
    return rho_0 * np.exp((h_0 - h) / H) * 10**-9 # kg/m^3

# assume atmosphere rotates with earth
# relative velocity of satellite with respect to atmosphere at location on earth
# with respect to inertial frame and expressed in body frame
def relative_vel(x, v):
    v_rel = np.array([v[0] + EARTH_ROTATION * x[1],
                      v[1] + EARTH_ROTATION * x[0],
                      v[2]])
    return v_rel # m/s

# drag equation, unit agnostic
def drag(rho, v, S):
    return -1/2 * rho * DRAG_COEFF * np.linalg.norm(v) * v * S

# gravitational torque. inputs in inertial frame, outputs in body frame
def gravity_torque(r_I, attitude_BI):
    r = np.linalg.norm(r_I)
    n = sandwich(attitude_BI, - r_I / r)
    return (3 * MU / r**3) * np.cross(n, (MOMENT_OF_INERTIA +
                                          WHEEL_MOMENT).dot(n)) # kg * m^2 / s^2

# gradient of 1st order term of IGRF model of magnetic potential
# see https://www.ngdc.noaa.gov/IAGA/vmod/igrf.html for relevant details
# be aware that every year, these approximated coefficients change a little
# see page 403 - 406 for details. i expect 20-50 uT magnitude
# takes position in ECEF frame, returns vector in ECEF frame, nanoTesla
# CHECK YOUR MATH, MAKE SURE SWITCHING FROM km TO m DIDN'T F IT UP
def magnetic_field(r):
    n_r = np.linalg.norm(r)
    m = MAG_REF_RADIUS**3 * np.array([G_1_1, H_1_1, G_1_0]) # magnetic dipole in ECEF
    return (3*np.dot(m, r) * r -
            m * n_r**2) / n_r**5 # nT

# kinematic equations
# ECEF ref
def position_derivative(lin_vel):
    return lin_vel

# quaternion kinematics for attitude
def attitude_derivative(attitude, body_ang_vel):
    return 1/2 * product(attitude, np.array([0,
                                    body_ang_vel[0],
                                    body_ang_vel[1],
                                    body_ang_vel[2]]))

# dynamic equations
# central force model for acceleration of satellite, ECEF ref
def lin_vel_derivative(position):
    g = (- MU / np.dot(position, position)) * normalize(position)
    return g # at some point, we'll include drag which depends on vel

# change in wheel velocity is just commanded for now
def wheel_vel_derivative(accl_cmd):
    return accl_cmd

# euler's rotational equation. body ref
def dw_dt(body_ang_vel, wheel_momenta, wheel_torques, external_torques):
    return INV_MOMENT_OF_INERTIA.dot(external_torques -
                                    wheel_torques -
                                    np.cross(body_ang_vel,
                                            MOMENT_OF_INERTIA.dot(body_ang_vel) +
                                             wheel_momenta))

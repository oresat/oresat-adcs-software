#!/usr/bin/env python3
from pydbus import SystemBus
from gi.repository import GLib
import time, random, datetime
import numpy as np
from geometry import * # I'm not sure if this still executes the file being imported from
from system_definition import *

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

class DynamicalSystem():
    def __init__(self, position, lin_vel, attitude, body_ang_vel, wheel_vel):
        self.full_state = np.array([position, lin_vel,
                                    attitude, body_ang_vel, wheel_vel])
        self.bus = SystemBus() # connect to bus
        self.dbus_client = self.bus.get("org.OreSat.ADCS")

    # kinematic equations
    # ECEF ref
    def position_derivative(self, lin_vel):
        return lin_vel

    # quaternion kinematics for attitude
    def attitude_derivative(self, attitude, body_ang_vel):
        return 1/2 * quat_product(attitude, np.array([0,
                                        body_ang_vel[0],
                                        body_ang_vel[1],
                                        body_ang_vel[2]]))

    # dynamic equations
    # central force model for acceleration of satellite, ECEF ref
    def lin_vel_derivative(self, position):
        g = (- MU / np.dot(position, position)) * normalize(position)
        return g # at some point, we'll include drag which depends on vel

    # change in wheel velocity is just commanded for now
    def wheel_vel_derivative(self, accl_cmd):
        return accl_cmd

    # euler's rotational equation. body ref
    def body_ang_vel_derivative(self, wheel_vel, body_ang_vel,
                                wheel_torques, external_torques):
        wheel_momenta = rw_sys_momentum(wheel_vel, body_ang_vel)
        return INV_MOMENT_OF_INERTIA.dot(external_torques -
                                        wheel_torques -
                                        np.cross(body_ang_vel,
                                                MOMENT_OF_INERTIA.dot(body_ang_vel) +
                                                 wheel_momenta))

    def linear_motion(self, position, lin_vel):
         return np.array([self.position_derivative(lin_vel),
                        self.lin_vel_derivative(position)])


    def rotational_motion(self, position, attitude, body_ang_vel, wheel_vel,
                            body_ang_accl, accl_cmd, magnetorquer_currents):
         T_gg = gravity_torque(position, attitude)
         T_rw = rw_sys_torque(accl_cmd, body_ang_accl)
         return np.array([self.attitude_derivative(attitude, body_ang_vel),
                          self.body_ang_vel_derivative(wheel_vel, body_ang_vel,
                                                        T_rw, T_gg)])

    def full_state_derivatives(self, position, lin_vel, attitude, body_ang_vel, wheel_vel,
                                body_ang_accl, accl_cmd, magnetorquer_currents):
        dxdt, dvdt = self.linear_motion(position, lin_vel)
        dqdt, dwdt = self.rotational_motion(position, attitude, body_ang_vel, wheel_vel,
                                            body_ang_accl, accl_cmd, magnetorquer_currents)
        dw_rw_dt = self.wheel_vel_derivative(accl_cmd)
        return np.array([dxdt, dvdt, dqdt, dwdt, dw_rw_dt])

    def publish_to_dbus(self):
        #self.full_state = np.array([position, lin_vel,
        #                            attitude, body_ang_vel, wheel_vel])
        # update the gps data
        self.dbus_client.GPS_Data = (
                 # postion x,y,z
                self.full_state[0],

                 # velocity x,y,z
                self.full_state[1],

                datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            )

        ra, dec, roll = quat_to_startracker(self.full_state[2])
        # update star tracker data
        self.dbus_client.StarTrackerData = (
                ra, dec, roll,
                datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                )


        # update magnetometer data
        self.dbus_client.MagnetometersData = [
                (
                random.randint(0, 100),
                datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                ),
                (
                random.randint(0, 100),
                datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                )
                ]

        # update reaction wheels data
        self.dbus_client.ReactionWheelsData = [
                (
                random.randint(0, 100),
                datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                ),
                (
                random.randint(0, 100),
                datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                ),
                (
                random.randint(0, 100),
                datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                ),
                (
                random.randint(0, 100),
                datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                )
                ]

        # update magnetorquer data
        self.dbus_client.MagnetorquerData = [
                (
                random.randint(0, 100),
                datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                )
                ]

class Integrator():
    def __init__(self, model, dt):
        self.model = model
        self.f = self.model.full_state_derivatives
        self.dt = dt
        self.history = [self.model.full_state, self.model.full_state]
        self.history_length = 5
    def RK4_step(self, state, param): # simplifying assumption to not update param for now
        k1 = self.f(*state, *param)
        k2 = self.f(*(state + k1 * self.dt/2), *param)
        k3 = self.f(*(state + k2 * self.dt/2), *param)
        k4 = self.f(*(state + k3 * self.dt), *param)
        return state + (k1 + 2*k2 + 2*k3 + k4) * self.dt / 6
    def update(self, state, param):
        next_step = self.RK4_step(state, param)
        if len(self.history) >= self.history_length:
            del self.history[0]
        self.history.append(next_step)
        self.model.full_state = next_step

    def derivative(self): # don't call this without a history or you'll crash
        return (self.history[-2] - self.history[-1]) / self.dt

    def integrate(self, duration, zero_order_hold):
        t = 0
        while duration > t + self.dt:
            body_ang_accl = self.derivative()[3]
            accl_cmd = zero_order_hold[0]
            magnetorquer_currents = zero_order_hold[1]
            self.update(self.model.full_state, [body_ang_accl, accl_cmd, magnetorquer_currents])
            t += self.dt
        #for i in self.history:
        #    print(i, '\n')
        #print("duration:", t, "seconds")



#test_sys = DynamicalSystem(x_0, v_0, q_0, w_0, whl_0)
#test_int = Integrator(test_sys, 0.1)
#test_int.integrate(10.0, [np.zeros(4), np.zeros(3)])

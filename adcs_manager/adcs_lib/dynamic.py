import numpy as np
from adcs_lib import frame, structure, environment, jday, quaternion

class DynamicalSystem():
    '''This class stores the state vector, Julian date, reference frame transformations,
    satellite structural model, and environmental models. It also calculates the vector field for the states.'''
    def __init__(self, position, lin_vel, attitude, body_ang_vel, wheel_vel, date_and_time):
        self.state       = np.array([position, lin_vel, attitude, body_ang_vel, wheel_vel], dtype=object)
        year, month, day, hour, minute, second = date_and_time
        self.clock       = jday.Clock(year, month, day, hour, minute, second)
        self.update_transformation_matrices()
        self.satellite   = structure.Satellite(structure.LENGTH, structure.WIDTH, structure.HEIGHT, structure.PRINCIPAL,
                                               structure.INCLINATION, structure.AZIMUTH, structure.PARALLEL, structure.ORTHOGONAL)
        self.enviro      = environment.Environment(self.satellite)

    def update_transformation_matrices(self):
        '''Some of the reference frame transformations depend on the current state vector.'''
        self.GCI_to_ECEF = frame.inertial_to_ecef(self.clock)

    def vector_field(self, position, lin_vel, attitude, body_ang_vel, wheel_vel,
                     body_ang_accl, mag_moment, whl_accl):
        '''This is the vector field for our state space. It defines the differential equation to be solved.'''
        attitude     = quaternion.normalize(attitude) # we do this for the intermediate RK4 steps
        F_env, T_env = self.enviro.env_F_and_T(position, lin_vel, attitude, self.GCI_to_ECEF, mag_moment)
        T_whl        = self.satellite.reaction_wheels.torque(whl_accl, body_ang_accl)
        H_whl        = self.satellite.reaction_wheels.momentum(wheel_vel, body_ang_vel)

        dxdt, dvdt   = (lin_vel, F_env / self.satellite.mass)
        dqdt         = 0.5 * quaternion.product(attitude, np.array([0, body_ang_vel[0], body_ang_vel[1], body_ang_vel[2]]))
        dwdt         = self.satellite.inv_red_moment.dot(T_env - T_whl - np.cross(body_ang_vel, H_whl + self.satellite.reduced_moment.dot(body_ang_vel)))
        dw_rw_dt     = whl_accl
        return np.array([dxdt, dvdt, dqdt, dwdt, dw_rw_dt], dtype=object)

class Integrator():
    '''This is a numerical integrator for propagating an initial state through state space.'''
    def __init__(self, model, dt):
        self.model         = model
        self.f             = self.model.vector_field
        self.dt            = dt
        self.last_ang_accl = np.zeros(3)

    def RK4_step(self, state, param):
        '''This is the 4th order Runge-Kutta method. We could exchange this method for any other algorithm.
        However, this is a nice general purpose tool. Error is on the order of dt^5.'''
        k1     = self.f(*state, *param)
        k2     = self.f(*(state + k1 * self.dt/2), *param)
        k3     = self.f(*(state + k2 * self.dt/2), *param)
        k4     = self.f(*(state + k3 * self.dt), *param)
        change = (k1 + 2*k2 + 2*k3 + k4) / 6
        return state + change * self.dt, change[-2]

    def update(self, state, param):
        '''This takes the model one step forward and updates its clock and transformations.'''
        next_step, self.last_ang_accl = self.RK4_step(state, param)
        next_step[2]                  = quaternion.normalize(next_step[2])
        self.model.state              = next_step
        self.model.clock.tick(self.dt)
        self.model.update_transformation_matrices()

    def integrate(self, duration, zero_order_hold):
        '''This is the front-facing interface for the library. It takes an integration duration and a set of fixed exogenous commands.
        Then it propagates the dynamic model for that duration using those commands.'''
        t = self.dt
        while t < duration or abs(t - duration) < 0.001:
            mag_moment = zero_order_hold[0]
            accl_cmd   = zero_order_hold[1]
            self.update(self.model.state, [self.last_ang_accl, mag_moment, accl_cmd])
            t += self.dt

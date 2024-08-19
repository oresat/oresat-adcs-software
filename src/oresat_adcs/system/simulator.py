import numpy as np
from ..functions import vector, quaternion
from ..classes import sensor, dynamics

class SimulatorDaemonInterface():
    '''This is the interface with the Simulator Daemon.
    The end user has the option to input control commands and propagate internal state for a duration,
    and also the option to get noisy sensor readings from the simulated environment.
    '''
    def __init__(self, dynamics_model, initial_state, dt):
        dt    = 0.05 # perhaps we want to choose this upstream?

        #self.model        = dynamic.DynamicalSystem(x_0, v_0, q_0, w_0, whl_0, t_0, satellite)
        self.model = dynamics_model
        self.integrator   = dynamics.Integrator(self.model, initial_state, dt)

    def input(self, duration, zero_order_hold):
        '''Takes a duration of time to progagate state by, and a tuple with magnetorquer current and wheel acceleration commands to hold for said duration.

        Parameters
            duration : float :The length of time (s) to integrate over.
            zero_order_hold : list : Two numpy.ndarrays, for current commands and acceleration commands.
        '''
        self.integrator.integrate(duration, zero_order_hold)

    def output(self, noisy):
        '''Returns measurements of the satellite's state, possibly with noise added.

        Parameters
            noisy : bool : True if measurements should have noise added.

        Returns
            numpy.ndarray : Array of arrays for state of satellite.
        '''
        return self.model.measurement(self.integrator.state, noisy)

    def propagate(self, duration, zero_order_hold):
        '''This function is for the daemon specifically so that it only needs one function call to use the library.

        Parameters
            duration : float : The length of time (s) to integrate over.
            zero_order_hold : list : Two numpy.ndarrays, for current commands and acceleration commands.

        Returns
        numpy.ndarray : Array of arrays for true state of satellite.
        '''
        self.input(duration, zero_order_hold)
        return self.output(noisy=False)

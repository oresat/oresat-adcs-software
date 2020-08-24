import numpy as np
from adcs_lib import dynamic, sensor

class SimulatorDaemonInterface():
    '''This is the interface with the Simulator Daemon.
    The end user has the option to input control commands and propagate internal state for a duration,
    and also the option to get noisy sensor readings from the simulated environment.'''
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    def __init__(self):
        # dynamic model initial conditions
        x_0   = np.array([5.581498e6, -3.881737e6, 1.421855e4])
        v_0   = np.array([2.708896e3, 3.914674e3, 5.994012e3])
        #q_0   = np.array([np.sqrt(2)/2, np.sqrt(2)/2, 0, 0])
        q_0   = np.array([1, 0,0 ,0])
        w_0   = np.array([0.08726646, 0.08726646, 0.08726646]) # 5 degrees/s / axis. worst case
        #w_0 = np.array([0.0, 0.0, 0.1])
        #w_0 = np.zeros(3)
        whl_0 = np.array([0, 0, 0, 0])
        t_0   = (2020, 9, 1, 0, 0, 0)
        dt    = 0.05 # perhaps we want to choose this upstream?
        noisy = False

        self.model        = dynamic.DynamicalSystem(x_0, v_0, q_0, w_0, whl_0, t_0)
        self.integrator   = dynamic.Integrator(self.model, dt)

    def input(self, duration, zero_order_hold):
        '''Takes a duration of time to progagate state by, and a tuple with wheel acceleration commands and magnetic moment command to hold for said duration.'''
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        self.integrator.integrate(duration, zero_order_hold)

    def output(self, noisy):
        '''Returns measurements of the satellite's state, possibly with noise added.'''
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        return self.model.measurement(noisy)

    def propagate(self, duration, zero_order_hold):
        '''This function is for the daemon specifically so that it only needs one function call to use the library.'''
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        self.input(duration, zero_order_hold)
        return self.output(noisy=False)

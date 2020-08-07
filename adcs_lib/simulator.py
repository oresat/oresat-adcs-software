import numpy as np
import dynamic, sensor

class SimulatorDaemonInterface():
    '''This is the interface with the Simulator Daemon.
    The end user has the option to input control commands and propagate internal state for a duration,
    and also the option to get noisy sensor readings from the simulated environment.'''
    def __init__(self):
        # dynamic model initial conditions
        #x_0 = np.array([1.91831688780483e6, 6.52089247589002e6, 1.94903609076208e3]) # m. this and v are from freeflyer
        #v_0 = np.array([-4.56009030296681e3, 1.33278201869975e3, 6.0067784327447e3]) # m/s
        x_0   = np.array([5.581498e6, -3.881737e6, 1.421855e4])
        v_0   = np.array([2.708896e3, 3.914674e3, 5.994012e3])
        q_0   = np.array([1, 0, 0, 0])
        w_0   = np.array([0.08726646, 0.08726646, 0.08726646]) # 5 degrees/s / axis. worst case
        #w_0 = np.array([0,0,0.1])
        whl_0 = np.array([0, 0, 0, 0])
        t_0   = (2020, 9, 1, 0, 0, 0)
        dt    = 0.05 # perhaps we want to choose this upstream?
        noisy = False

        self.model        = dynamic.DynamicalSystem(x_0, v_0, q_0, w_0, whl_0, t_0)
        self.integrator   = dynamic.Integrator(self.model, dt)
        self.gyro         = sensor.Gyro(0, 5.818 * 10**(-4), noisy, self.model)
        self.magnetometer = sensor.Magnetometer(0, 0.05e-6, noisy, self.model)
        self.startracker  = sensor.StarTracker(0, 1.5 * 10**(-5), noisy, self.model)
        self.gps_pos      = sensor.GPS_pos(0, 30, noisy, self.model)
        self.gps_vel      = sensor.GPS_vel(0, 2, noisy, self.model)

    def input(self, duration, zero_order_hold):
        '''Takes a duration of time to progagate state by, and a tuple with wheel acceleration commands and magnetic moment command to hold for said duration.'''
        self.integrator.integrate(duration, zero_order_hold)

    def output(self):
        '''Returns measurements of the satellite's state, possibly with noise added.'''
        x = self.gps_pos.measurement()
        v = self.gps_vel.measurement()
        q = self.startracker.measurement()
        w = self.gyro.measurement()
        W = self.model.state[4]
        B = self.magnetometer.measurement()
        return (x, v, q, w, W, B)

    def propagate(self, duration, zero_order_hold):
        '''This function is for the daemon specifically so that it only needs one function call to use the library.'''
        self.input(duration, zero_order_hold)
        return self.output()

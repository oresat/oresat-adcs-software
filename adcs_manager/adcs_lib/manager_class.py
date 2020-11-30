import numpy as np
from adcs_lib import controller, observer
from adcs_lib.state_machine import State

class ManagerDaemonInterface():
    '''
    External interface for core ADCS functionality to simplify system for CS people to manage.
    Everything here is still highly tentative.

    Parameters
    ----------
    gyro_step_size : float
        Length to step Kalman filter forward by.
    gps_step_size : float
        Length to step Kalman filter forward by.
    truth_model : dynamic.DynamicalSystem
        Placeholder model for SITL testing, standing in for actual sensors.
    '''
    def __init__(self, gyro_step_size, gps_step_size, truth_model):
        self.filter               = observer.KalmanFilters(gyro_step_size, gps_step_size, truth_model)
        self.mag_controller       = controller.MagnetorquerController(self.filter.PosFilter.model, bang_bang=False)
        self.rw_controller        = controller.ReactionWheelsController(self.filter.PosFilter.model, 0.707, 0.1)
        self.mag_cmd, self.rw_cmd = np.zeros(3), np.zeros(4)

    def mission_input(self, mission_data):
        '''
        Receives high-level parameters defining the current mission state, then
        calculates and stores appropriate actuator commands to achieve goal.
        Mission data format is preliminary still.
        This method may eventually subsume the state handling logic in the main manager.py

        Parameters
        ----------
        mission_data : list
            State machine ID # (int), inertial-frame target point (numpy.ndarray), body-frame boresight vector (numpy.ndarray)
        '''
        model_state = self.filter.output()
        if mission_data[0] == State.SLEEP.value or mission_data[0] == State.FAILED.value:
            self.mag_cmd = np.zeros(3)
            self.rw_cmd  = np.zeros(4)

        elif mission_data[0] == State.DETUMBLE.value:
            self.mag_cmd = self.mag_controller.detumble(model_state)
            self.rw_cmd  = np.zeros(4)

        elif mission_data[0] == State.POINT.value:
            self.mag_cmd = np.zeros(3)
            self.rw_cmd  = self.rw_controller.point_and_stare(model_state, mission_data)

        #elif mission_data[0] == State.BBQ.value:
        #    self.mag_cmd = self.mag_controller.no_q_bbq_roll(model_state)
        #    self.rw_cmd  = np.zeros(4)
        elif mission_data[0] == State.BBQ.value:
            self.mag_cmd = self.mag_controller.desaturate(model_state, np.zeros(4))
            self.rw_cmd  = self.rw_controller.induce_bbq_roll_attitude(model_state, self.filter.PosFilter.model.satellite.magnetorquers.actuate(self.mag_cmd))
        elif mission_data[0] == State.POWERDOWN.value:
            self.mag_cmd = np.zeros(3)
            null = np.array([-0.5, 0.5, -0.5, 0.5])
            mult = self.mag_controller.k_bbq * 250
            self.rw_cmd = - np.dot(model_state[4], null) * null
            if np.linalg.norm(self.rw_cmd) < 0.00001:
                self.rw_cmd = - model_state[4]
                if np.linalg.norm(model_state[4]) < 0.01:
                    self.rw_cmd = np.zeros(4)
                    self.mag_cmd = np.zeros(3)
                else:
                    self.rw_cmd *= mult
                    self.mag_cmd = np.zeros(3)
                    #self.mag_cmd = self.mag_controller.actuator_commands(model_state,
                        #-self.filter.PosFilter.model.satellite.reaction_wheels.torque(self.rw_cmd))
            else:
                self.mag_cmd = np.zeros(3)
            #self.rw_cmd = np.zeros(4)
            #print(self.rw_cmd)
            #print(np.linalg.norm(model_state[4]))

    def sensor_input(self, sensor_data):
        '''
        Updates the Kalman filters when new data comes in from sensors.
        May want to decouple the two filters.
        May want to have sensor_data only contain position/attitude data.
        May want to have filters be updated by dbus signals.

        Parameters
        ----------
        sensor_data : numpy.ndarray
            Array of arrays for present state of satellite.
        '''
        if sensor_data is not None:
            self.filter.update(sensor_data)

    def output(self):
        '''
        Returns actuators commands and estimated state.

        Returns
        -------
        numpy.ndarray
            Magnetorquer current (A) commands.
        numpy.ndarray
            Reaction wheel acceleration (rad/s^2) commands.
        numpy.ndarray
            Estimated state of satellite.
        '''
        return self.mag_cmd, self.rw_cmd, self.filter.output()

    def propagate(self, duration, mission_data, sensor_data):
        '''This is for the daemon specifically so that it only needs one function call to use the library.
        Given a length of time, a mission to execute, and optionally sensor data,
        updates filters, calculates commands, and propagates filter forward.
        May want these to be asynchronous.

        Parameters
        ----------
        duration : float
            Length of time to propagate filters forward by.
        mission_data : list
            State machine ID # (int), inertial-frame target point (numpy.ndarray), body-frame boresight vector (numpy.ndarray)
        sensor_data : numpy.ndarray
            Array of arrays for present state of satellite.

        Returns
        -------
        numpy.ndarray
            Magnetorquer current (A) commands.
        numpy.ndarray
            Reaction wheel acceleration (rad/s^2) commands.
        numpy.ndarray
            Estimated state of satellite.
        '''
        self.sensor_input(sensor_data)
        self.mission_input(mission_data)
        self.filter.propagate(duration)
        return self.output()

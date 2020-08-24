import numpy as np
from adcs_lib import controller, observer
from adcs_lib.state_machine import State

# this entire file is a pile of kludge, just trying to get something working

class ManagerDaemonInterface():
    '''

    Parameters
    ----------

    Returns
    -------
    '''
    def __init__(self, gyro_step_size, gps_step_size, truth_model):
        self.filter = observer.KalmanFilters(gyro_step_size, gps_step_size, truth_model)
        self.mag_controller = controller.MagnetorquerController(self.filter.PosFilter.model.satellite.magnetorquers, bang_bang=False)
        self.rw_controller  = controller.ReactionWheelsController(self.filter.PosFilter.model, .707, 0.025)
        self.mag_cmd, self.rw_cmd = np.zeros(3), np.zeros(4)

    def mission_input(self, mission_data):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        model_state = self.filter.output()
        if mission_data[0] == State.SLEEP.value or mission_data[0] == State.FAILED.value:
            self.mag_cmd, self.rw_cmd = np.zeros(3), np.zeros(4)

        elif mission_data[0] == State.DETUMBLE.value:
            self.mag_cmd = self.mag_controller.detumble(model_state)
            self.rw_cmd  = np.zeros(4)

        elif mission_data[0] == State.POINT.value:
            self.mag_cmd = np.zeros(3)
            q_cmd        = self.rw_controller.point_and_stare(model_state, mission_data)
            u = self.rw_controller.tracking(model_state, [np.zeros(3), q_cmd])
            self.rw_cmd  = self.rw_controller.transform_law_to_wheels(u)

        elif mission_data[0] == State.BBQ.value:
            #self.mag_cmd = np.zeros(3)
            u = self.rw_controller.induce_bbq_roll(model_state)
            #u        = self.rw_controller.exit_mission_mode(model_state)
            self.mag_cmd = self.mag_controller.actuator_commands(model_state, u)
            self.rw_cmd  = np.zeros(4)
            #self.rw_cmd  = self.rw_controller.transform_law_to_wheels(u)

    def sensor_input(self, sensor_data):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        if sensor_data is not None:
            self.filter.update(sensor_data)

    def output(self):
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        return self.mag_cmd, self.rw_cmd, self.filter.output()

    def propagate(self, duration, mission_data, sensor_data):
        '''This is for the daemon specifically so that it only needs one function call to use the library.'''
        '''

        Parameters
        ----------

        Returns
        -------
        '''
        self.sensor_input(sensor_data)
        self.mission_input(mission_data)
        self.filter.propagate(duration)
        return self.output()

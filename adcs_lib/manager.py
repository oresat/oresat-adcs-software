import numpy as np
import controller, observer

# this entire file is a pile of kludge, just trying to get something working

class ManagerDaemonInterface():
    def __init__(self, dummy_model):
        self.filter = observer.DummyFilter(dummy_model)
        self.mag_controller = controller.MagnetorquerController()
        self.rw_controller  = controller.ReactionWheelsController(self.filter.model.satellite.reduced_moment, .7, 100)
        self.mag_cmd, self.rw_cmd = np.zeros(3), np.zeros(4)

    def mission_input(self, mission_data):
        state, mag_field = self.filter.output()
        if mission_data[0] == 0:
            self.mag_cmd, self.rw_cmd = np.zeros(3), np.zeros(4)
        elif mission_data[0] == 1:
            self.mag_cmd, self.rw_cmd = self.mag_controller.detumble(mag_field, state[3]), np.zeros(4)
        elif mission_data[0] == 2:
            self.mag_cmd = np.zeros(3)
            self.rw_cmd = self.rw_controller.point_and_stare(state[0], state[1], state[3], state[2], mission_data[2], mission_data[1])

    def sensor_input(self, sensor_data):
        self.filter.update(sensor_data)

    def output(self):
        return self.mag_cmd, self.rw_cmd, self.filter.output()

    def propagate(self, duration, mission_data, sensor_data):
        '''This is for the daemon specifically so that it only needs one function call to use the library.'''
        self.sensor_input(sensor_data)
        self.mission_input(mission_data)
        self.filter.propagate(duration, [self.mag_cmd, self.rw_cmd])
        return self.output()

import numpy as np
from geometry import *

class MagnetorquerController:
    def __init__(self):
        #self.last_measurement = None # when done detumbling, should probably reset this
        self.first_time = True
        # approximate control gain, see page 310 formula. at some point do calculation for magnetic equator instead of actual
        self.gain = 1.784127 * 5.09404743e-3 * 4 * np.pi / 5560.8

    # bdot control for detumbling. we'll want something only slightly different for control in general
    # could also bang-bang this, see pg 309
    def detumble(self, adcs_data):
        current_mag, last_mag = adcs_data[3]
        if self.first_time: # first time
            magnetorquer_command = np.array((0.0, 0.0, 0.0))
            self.first_time = False
        else:
            bdot = (normalize(np.array(current_mag[0])) - normalize(np.array(last_mag[0]))) / 0.05 # very inaccurate, placeholder basically
            magnetorquer_command = - self.gain * bdot / np.linalg.norm(np.array(current_mag[0]))
        return magnetorquer_command

    # general tracking/regulation for when we have access to sensors
    # we will need a second function for calculating the set point based on
    # which state and mode we're in
    def tracking(self, adcs_data, set_point):
        current_mag, _ = adcs_data[3]
        current_mag = np.array(current_mag[0])
        angular_rate = adcs_data[2][0]
        #ra, dec, roll, _ = adcs_data[1]
        magnetorquer_command = -self.gain / np.linalg.norm(current_mag)**2  * np.cross(current_mag, angular_rate - set_point)
        return magnetorquer_command



    def point(self, adcs_data):

        # Do stuff

        magnetorquer_command = np.array((0.0, 0.0, 0.0))
        return magnetorquer_command

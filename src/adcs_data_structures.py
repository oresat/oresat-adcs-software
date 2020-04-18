"""
"""

# ECEF
class GPS_Data:
    def __init__(self):
        self.position_x = 0.0 # m/s
        self.position_y = 0.0 # m/s
        self.position_z = 0.0 # m/s
        self.velocity_x = 0.0 # m/s^2
        self.velocity_y = 0.0 # m/s^2
        self.velocity_z = 0.0 # m/s^2
        self.timestamp = "" # linux time


class StarTrackerData:
    def __init__(self):
        self.right_ascension = 0.0
        self.declination = 0.0
        self.orientation = 0.0
        self.timestamp = "" # linux time


class MagnometerData:
    def __init__(self):
        self.timestamp = "" # linux time


class ReactionWheelData:
    def __init__(self):
        self.timestamp = "" # linux time


class MagnetorquerData:
    def __init__(self):
        self.timestamp = "" # linux time


class ADCS_Data:
    def __init__(self):
        self.gps_data = GPS_Data()
        self.st_data = StarTrackerData()
        self.magnetorquer_data = MagnetorquerData()
        self.magnometer_pos_z = MagnometerData()
        self.magnometer_neg_z = MagnometerData()
        self.magnetorquer_data = MagnometerData()
        self.reaction_wheels = []
        for x in range(4):
            self.reaction_wheels.append(ReactionWheelData())


from datetime import datetime


ACS1 = 0 # RW: +x +y, Mag: Z
ACS2 = 1 # RW: -x +y, Mag: X
ACS3 = 2 # RW: -x -y, Mag: Y
ACS4 = 3 # RW: +x -y


class ACSData():
    """
    This class holds the attributes for a single ACS board.
    """

    def __init__(self):
        self.position = 0           #: Reaction wheel position. **Type:** int
        self.velocity = 0           #: Reaction wheel (angular) velocity. **Type:** int
        self.temperature = 0        #: Reaction wheel temperature. **Type:** int
        self.last_update_dt = None  #: Timestamp for when the board was last updated. **Type:** datetime


class BoardData():
    """
    This POD class holds all data on the CANbus from a ADCS board.
    """

    def __init__(self):
        # SDR GPS board data (state vector)
        self.position = [0.0, 0.0, 0.0] #: Position of satellite in km using ECEF coordinates. **Type:** [float]
        self.velocity = [0.0, 0.0, 0.0] #: Satellite's velocity in km/s XYZ. **Type:** [float]
        self.pos_vel_timestamp = None   #: Timestamp for position and velocity from SDR GPS. **Type:** datetime

        # Star Tracker board data (celestial coordinates)
        self.right_ascension = 0.0              #: Satellite's right ascension in degrees. **Type:** [float]
        self.declination = 0.0                  #: Satellite's declination in degrees. **Type:** [float]
        self.orientation = 0.0                  #: Satellite's orientation in degrees. **Type:** [float]
        self.celestial_coor_timestamp = None    #: Timestamp for the celestial coordinates from Star Tracker. **Type:** datetime

        # IMU and Magnetometer board data
        # acceleration = [0, 0, 0] # xyz
        self.angular_velocity = [0, 0, 0]       #: Satellite's angular velocity in rad/second XYZ. **Type:** [float]
        # NOTE / TODO need Magnetometer data?
        self.mag_field_vector0 = [0, 0, 0]      #: IMU-Magnetometer magnetic field vector (1 of 2). **Type:** [float]
        self.mag_field_vector1 = [0, 0, 0]      #: IMU-Magnetometer magnetic field vector (2 of 2). **Type:** [float]
        self.imu_last_update_dt = None          #: Datetime when the IMU-Magnetorquer data was last updated. **Type:** datetime

        # ACS board data
        self.reaction_wheels_data = [0, 0, 0, 0]        #: description TBD **Type:** TBD
        self.magnetorquer_data = [0, 0, 0]              #: description TBD **Type:** TBD
        self.acs_data = [ACSData() for _ in range(4)]   #: description TBD **Type:** [ACSData]

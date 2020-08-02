from datetime import datetime


ACS1 = 0 # RW: +x +y, Mag: Z
ACS2 = 1 # RW: -x +y, Mag: X
ACS3 = 2 # RW: -x -y, Mag: Y
ACS4 = 3 # RW: +x -y


class ACSData():
    """
    This class holds the attributes for a single ACS board.

    Attributes
    ----------
    position : int
        Reaction wheel position.
    velocity : int
        Reaction wheel (angular) velocity.
    temperature : int
        Reaction wheel temperature.
    last_update_dt : datetime
        Timestamp for when the board was last updated.
    """

    position = 0
    velocity = 0
    temperature = 0
    last_update_dt = None


class BoardData():
    """
    This POD class should hold all data on the CANbus from a ADCS board.

    Attributes
    ----------
    position : [float]
        Position of satelitte in km using ECEF coordinates.
    velocity : [float]
        Satellite's velocity in km/s XYZ.
    pos_vel_timestamp : datetime
        timestamp for position and velocity from SDR GPS.
    right_ascension : float
        Satellite's right ascension in degrees.
    declination : float
        Satellite's declination in degrees.
    orientation : float
        Satellite's orientation in degrees.
    celestial_coor_timestamp : datetime
        timestamp for the celestial coordinates from Star Tracker.
    acceleration : [float]
        Satellite's acceleration in km/s^2 XYZ.
    angular_velocity : [float]
        Satellite's angular velocity in rad/second XYZ.
    mag_field_vector0 : [int]
        IMU-Magnetometer magnetic field vector (1 of 2).
    mag_field_vector1 : [int]
        IMU-Magnetometer magnetic field vector (2 of 2).
    imu_last_update_dt : datetime
        Datetime when the imu and magnetorquer data was last updated.
    reaction_wheels_data : [TODO]
        TODO
    magnetorquer_data : [TODO]
        TODO
    acs_data : [ACSData]
        List of containers for ACS board data.
    adc_last_update_dt : [datetime]
        List of datetimes when an adc board's data was last updated. One
        datetime for each board.
    """

    # SDR GPS board data (state vector)
    position = [0.0, 0.0, 0.0] # xyz
    velocity = [0.0, 0.0, 0.0] # xyz
    pos_vel_timestamp = None

    # Star Tracker board data (celestial coordinates)
    right_ascension = 0.0
    declination = 0.0
    orientation = 0.0
    celestial_coor_timestamp = None

    # IMU and Magnetometer board data
    # acceleration = [0, 0, 0] # xyz
    angular_velocity = [0, 0, 0] # xyz
    # NOTE / TODO need Magnetometer data?
    mag_field_vector0 = [0, 0, 0]
    mag_field_vector1 = [0, 0, 0]
    imu_last_update_dt = None

    # ACS board data
    reaction_wheels_data = [0, 0, 0, 0] # TODO replace with real data
    magnetorquer_data = [0, 0, 0] # TODO replace with real data
    acs_data = [ACSData() for _ in range(4)]

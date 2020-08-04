from pydbus.generic import signal
import threading
import time
from datetime import datetime
from board_data import BoardData
from state_machine import State, StateMachine

class DbusServer(object):
    """
    Defines the dbus interface for the ADCS Manager.
    Attributes
    ----------
    dbus : str
        The XML definition of the dbus interface. Required by pydbus.
    _data_lock : Lock
        Mutex for accessing any data control by this class.
    _board_data : BoardData
        Holds all the data for each ADCS board.
    """

    dbus = """
        <node>
            <interface name="org.OreSat.ADCSManager">
                <method name="ChangeMode">
                    <arg name="new_mode" type="i" direction="in"/>
                </method>
                <method name="NewACSData">
                    <arg name="board_number" type ="i" direction="in"/>
                    <arg name="velocity" type="n" direction="in"/>
                    <arg name="position" type="n" direction="in"/>
                    <arg name="temperature" type="n" direction="in"/>
                    <arg name="timestamp" type="s" direction="in"/>
                </method>
                <method name="NewIMUMagData">
                    <arg name="angular_velocity" type="(nnnn)" direction="in"/>
                    <arg name="mag_field_vector0" type="(nnn)" direction="in"/>
                    <arg name="mag_field_vector1" type="(nnn)" direction="in"/>
                </method>
                <method name="NewGPSData">
                    <arg name="position" type="(ddd)" direction="in"/>
                    <arg name="velocity" type="(ddd)" direction="in"/>
                    <arg name="timestamp" type="s" direction="in"/>
                    <arg name="output" type="b" direction="out"/>
                </method>
                <method name="NewStarTrackerData">
                    <arg name="right_ascension" type="d" direction="in"/>
                    <arg name="declination" type="d" direction="in"/>
                    <arg name="orientation" type="d" direction="in"/>
                    <arg name="timestamp" type="s" direction="in"/>
                    <arg name="output" type="b" direction="out"/>
                </method>
                <method name="NewMagCommands">
                    <arg name="mag_x_command" type="i" direction="in"/>
                    <arg name="mag_y_command" type="i" direction="in"/>
                    <arg name="mag_z_command" type="i" direction="in"/>
                </method>
                <method name="NewRWCommands">
                    <arg name="rw1_command" type="i" direction="in"/>
                    <arg name="rw2_command" type="i" direction="in"/>
                    <arg name="rw3_command" type="i" direction="in"/>
                    <arg name="rw4_command" type="i" direction="in"/>
                </method>
                <method name="LastRWCommand">
                    <arg name="rw_number" type="i" direction="in"/>
                    <arg name="output" type="i" direction="out"/>
                </method>
                <property name="CurrentMode" type="i" access="read"/>
                <property name="PointCoordinates" type="ddd" access="readwrite"/>
                <property name="GPSStateVector" type="(ddd)(ddd)s" access="read"/>
                <property name="STCelestialCoordinates" type="ddds" access="read"/>
                <property name="IMUAcceleration" type="(nnn)" access="read"/>
                <property name="IMUAngularVelocity" type="(nnn)" access="read"/>
                <property name="LastMagXCommand" type="i" access="read"/>
                <property name="LastMagYCommand" type="i" access="read"/>
                <property name="LastMagZCommand" type="i" access="read"/>
                <signal name="ReactionWheelsCommand">
                    <arg name="rw1_command" type="i"/>
                    <arg name="rw2_command" type="i"/>
                    <arg name="rw3_command" type="i"/>
                    <arg name="rw4_command" type="i"/>
                </signal>
                <signal name="MagnetorquerCommand">
                    <arg name="mag_x_command" type="i"/>
                    <arg name="mag_y_command" type="i"/>
                    <arg name="mag_z_command" type="i"/>
                </signal>
                <signal name="VisualizationDataSignal">
                    <arg name="sun_pointing_unit_vector" type="(ddd)"/>
                    <arg name="magnetic_field_vector" type="(ddd)"/>
                    <arg name="angular_momentum" type="(ddd)"/>
                    <arg name="angular_velocity" type="(ddd)"/>
                    <arg name="spacecraft_attitude" type="(dddd)"/>
                    <arg name="central_orbit_position" type="(ddd)"/>
                    <arg name="central_orbit_velocity" type="(ddd)"/>
                </signal>
            </interface>
        </node>
        """

    # dbus signals
    MagnetorquerCommand = signal()
    ReactionWheelsCommand = signal()
    VisualizationDataSignal = signal()

    # mutex
    _data_lock = threading.Lock()

    # board data
    _board_data = BoardData()

    # state machine
    _sm = StateMachine()

    # debug values
    _last_rw_commands = [0, 0, 0, 0]
    _last_mag_x_command = 0
    _last_mag_y_command = 0
    _last_mag_z_command = 0


    def ChangeMode(self, new_mode):
        """
        Request a mode change. Will be handle asynchronosly by daemon class.

        Parameters
        ----------
        new_mode : int
            The new mode to go into.
        """

        self._data_lock.acquire()
        self.new_mode = new_mode
        self._data_lock.release()


    def NewACSData(self, board_number, velocity, position, temperature, timestamp):
        """
        Receive new data from an ACS board and update the ADCS board.

        Parameters
        ----------
        board_number : int
            The ACS board being read from (0-3).
        velocity : int
            ACS velocity.
        position : int
            ACS position.
        temperature : int
            ACS temperature.
        timestamp : str
            Timestamp of the most recent ACS data.
        """

        self._data_lock.acquire()
        self._board_data.acs_data[board_number].velocity = velocity
        self._board_data.acs_data[board_number].position = position
        self._board_data.acs_data[board_number].temperature = temperature
        self._board_data.acs_data[board_number].last_update_dt = timestamp
        self._data_lock.release()


    def NewIMUMagData(self, angular_velocity, mag_field_vector0, mag_field_vector1):
        """
        Receive new data from the IMU-Magnetometer board and update the ADCS board.

        Parameters
        ----------
        angular_velocity : [double]
            IMU-Mag angular velocity.
        mag_field_vector0 : [double]
            IMU-Mag magnetic field vector (1 of 2).
        mag_field_vector1 : [double]
            IMU-Mag magnetic field vector (2 of 2).
        """

        self._data_lock.acquire()
        self._board_data.angular_velocity = angular_velocity
        self._board_data.mag_field_vector0 = mag_field_vector0
        self._board_data.mag_field_vector1 = mag_field_vector1
        self._data_lock.release()


    def NewGPSData(self, position, velocity, timestamp):
        """
        Receive new data from the GPS board and update the ADCS board.

        Parameters
        ----------
        position : [double]
            GPS position.
        velocity : [double]
            GPS velocity.
        timestamp : str
            Timestamp of the most recent GPS data.
        """
        # TODO: include output

        self._data_lock.acquire()
        self._board_data.position = position
        self._board_data.velocity = velocity
        self._board_data.pos_vel_timestamp = timestamp
        self._data_lock.release()


    def NewStarTrackerData(self, right_ascension, declination, orientation, timestamp):
        """
        Receive new data from the Star Tracker board and update the ADCS board.

        Parameters
        ----------
        right_ascension : double
            Star tracker right ascension.
        declination : double
            Star tracker declination.
        orientation : double
            Star tracker orientation.
        timestamp : str
            Timestamp of the most recent Star Tracker data.
        """
        # TODO: include output

        self._data_lock.acquire()
        self._board_data.right_ascension = right_ascension
        self._board_data.declination = declination
        self._board_data.orientation = orientation
        self._board_data.celestial_coor_timestamp = timestamp
        self._data_lock.release()


    def NewMagCommands(self, mag_x_command, mag_y_command, mag_z_command):
        """
        Update the most recent set of magnetorquer commands.

        Parameters
        ----------
        mag_x_command : int
            Last magnetorquer X command.
        mag_y_command : int
            Last magnetorquer Y command.
        mag_z_command : int
            Last magnetorquer Z command.
        """

        self._data_lock.acquire()
        self._last_mag_x_command = mag_x_command
        self._last_mag_y_command = mag_y_command
        self._last_mag_z_command = mag_z_command
        self._data_lock.release()


    def NewRWCommands(self, rw1_command, rw2_command, rw3_command, rw4_command):
        """
        Update the most recent set of reaction wheel commands.

        Parameters
        ----------
        rw1_command : int
            Last reaction wheel 1 command.
        rw1_command : int
            Last reaction wheel 2 command.
        rw1_command : int
            Last reaction wheel 3 command.
        rw1_command : int
            Last reaction wheel 4 command.
        """

        self._data_lock.acquire()
        self._last_rw_commands = [rw1_command, rw2_command, rw3_command, rw4_command]
        self._data_lock.release()


    def LastRWCommand(self, rw_number):
        """
        Gets the most recent reaction wheel command for a given wheel number.

        Parameters
        ----------
        rw_number : int
            Number of the reaction wheel being read from (1-4).
        """
        # TODO: include output

        self._data_lock.acquire()
        rwc = self._last_rw_commands[rw_number - 1]
        self._data_lock.release()

        return rwc


    @property
    def CurrentMode(self):
        """
        Current mode getter.
        """
        # TODO: return the current mode, not state
        return self._sm.get_current_state()


    @property
    def PointCoordinate(self):
        """
        Point and stare coordinate getter.
        """
        return self._sm.get_current_state()


    @PointCoordinate.setter
    def PointCoordinate(self, coor):
        """
        Point and stare coordinate setter.
        Parameters
        ----------
        coor : [float]
            List of float for next point and stare maneuver. In the format of
            [x, y, z] for ECEF coordinates.
        """

        self._data_lock.acquire()
        self._point_coordinate = coor
        self._data_lock.release()


    @property
    def GPSStateVector(self):
        """
        Getter for the State Vector (GPS data).
        """

        self._data_lock.acquire()
        gps_data = (
                self._board_data.position,
                self._board_data.velocity,
                self._board_data.pos_vel_timestamp.isoformat()
                )
        self._data_lock.release()

        return gps_data


    @property
    def STCelestialCoordinates(self):
        """
        Getter for the star tracker celestial coordinates.
        """

        self._data_lock.acquire()
        st_data = (
            self._board_data.right_ascension,
            self._board_data.declination,
            self._board_data.orientation,
            self._board_data.celestial_coor_timestamp.isoformat()
        )
        self._data_lock.release()

        return st_data


    @property
    def IMUAcceleration(self):
        """
        Getter for the satellite's acceleration.
        """

        self._data_lock.acquire()
        imu_a = self._board_data.acceleration
        self._data_lock.release()

        return imu_a


    @property
    def IMUAngularVelocity(self):
        """
        Getter for the satellite's angular velocity.
        """

        self._data_lock.acquire()
        imu_av = self._board_data.angular_velocity
        self._data_lock.release()

        return imu_av
 

    @property
    def LastMagXCommand(self):
        """
        Getter for the last magnetorquer command's x value.
        """

        self._data_lock.acquire()
        magx = self._last_mag_x_command
        self._data_lock.release()

        return magx

    @property
    def LastMagYCommand(self):
        """
        Getter for the last magnetorquer command's x value.
        """

        self._data_lock.acquire()
        magy = self._last_mag_y_command
        self._data_lock.release()

        return magy


    @property
    def LastMagZCommand(self):
        """
        Getter for the last magnetorquer command's x value.
        """

        self._data_lock.acquire()
        magz = self._last_mag_z_command
        self._data_lock.release()

        return magz

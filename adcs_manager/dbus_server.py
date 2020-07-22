from pydbus.generic import signal
from threading import Lock
from datetime import datetime
from board_data import BoardData


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
                    <arg name="rw_tbd" type="i" direction="in"/>
                    <arg name="mag_tbd" type="i" direction="in"/>
                </method>
                <method name="NewIMUMagData">
                    <arg name="acceleration" type="(nnn)" direction="in"/>
                    <arg name="angular_velocity" type="(nnn)" direction="in"/>
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
                    <arg name="orientation" type="d" direction="im"/>
                    <arg name="timestamp" type="s" direction="in"/>
                    <arg name="output" type="b" direction="out"/>
                </method>
                <property name="CurrentMode" type="i" access="read"/>
                <property name="PointCoordinates" type="ddd" access="readwrite"/>
                <property name="GPSStateVector" type="(ddd)(ddd)s" access="read"/>
                <property name="STCelestialCoordinates" type="ddds" access="read"/>
                <property name="IMUAcceleration" type="(nnn)" access="read"/>
                <property name="IMUAngularVelocity" type="(nnn)" access="read"/>
                <property name="LastRW1Command" type="i" access="read"/>
                <property name="lastRW2Command" type="i" access="read"/>
                <property name="LastRW3Command" type="i" access="read"/>
                <property name="LastRW4Command" type="i" access="read"/>
                <property name="LastMagXCommand" type="i" access="read"/>
                <property name="LastMagYCommand" type="i" access="read"/>
                <property name="LastMagZCommand" type="i" access="read"/>
                <signal name="ReactionWheelsCommand">
                    <arg name="rw1_command" type="i"/>
                    <arg name="rw2_command" type="i"/>
                    <arg name="rw3_command" type="i"/>
                    <arg name="rw4_command" type="i"/>
                <signal name="MagnetorquerCommand">
                    <arg name="mag_x_command" type="i"/>
                    <arg name="mag_y_command" type="i"/>
                    <arg name="mag_z_command" type="i"/>
                </signal>
            </interface>
        </node>
        """

    # dbus signals
    MagnetorquerCommand = signal()
    ReactionWheelsCommand = signal()

    # mutex
    _data_lock = Lock()

    # board data
    _board_data = BoardData()


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


    @property
    def CurrentMode(self):
        """
        Current mode getter.
        """
        return self._sm.current_mode()


    @property
    def PointCoordinate(self):
        """
        Point and stare coordinate getter.
        """
        return self._sm.current_state()


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
    def StateVector(self):
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

        return temp



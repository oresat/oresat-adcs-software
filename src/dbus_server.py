from pydbus.generic import signal
from state_machine import StateMachine
import threading, datetime


class DbusServer(object):
    dbus = """
    <node>
        <interface name="org.OreSat.ADCS">
            <signal name="ReactionWheelsCommand">
                <arg type="ii" name="command" />
            </signal>
            <signal name="MagnetorquerCommand">
                <arg type="ii" name="command" />
            </signal>
            <property name="CurrentState" type="i" access="readwrite"/>
            <property name="GPS_Data" type="((ddd)(ddd)s)" access="readwrite"/>
            <property name="StarTrackerData" type="(ddds)" access="readwrite"/>
            <property name="MagnetometersData" type="a(is)" access="readwrite"/>
            <property name="ReactionWheelsData" type="a(is)" access="readwrite"/>
            <property name="MagnetorquerData" type="a(is)" access="readwrite"/>
        </interface>
    </node>
    """ # this wont work in __init__()

    # dbus signals
    MagnetorquerCommand = signal()
    ReactionWheelsCommand = signal()

    def __init__(self, state_machine_input):
        """
        Contructor.
        Pass in the state machine
        """

        self._lock = threading.Lock()
        self._state_machine = state_machine_input

        # initize tuples. must match xml type field
        time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self._gps_data = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), time)
        self._st_data = (0.0, 0.0, 0.0, time)
        self._magnetometers_data = [(0, time), (0, time)]
        self._reaction_wheels_data = [(0, time), (0, time), (0, time), (0, time)]
        self._magnetorquer_data = (0, time)


    @property
    def CurrentState(self):
        # wrapper for state machine getter.
        return self._state_machine.get_current_state()


    @CurrentState.setter
    def CurrentState(self, new_state):
        # wrapper for state machien setter.
        self._state_machine.change_state(new_state)


    # ------------------------------------------------------------------------
    # GPS


    @property
    def GPS_Data(self):
        """
        Getter for GPS data.
        Is used by dbus and can be use by other classes.
        """

        self._lock.acquire()
        temp = self._gps_data
        self._lock.release()

        return temp


    @GPS_Data.setter
    def GPS_Data(self, input_data):
        """
        Setter for GPS data.
        Is used by dbus.
        """

        self._lock.acquire()
        self._gps_data = input_data
        self._lock.release()


    # ------------------------------------------------------------------------
    # Star Tracker


    @property
    def StarTrackerData(self):
        """
        Getter for Star Tracker data.
        Is used by dbus and can be use by other classes.
        """

        self._lock.acquire()
        temp = self._st_data
        self._lock.release()

        return temp


    @StarTrackerData.setter
    def StarTrackerData(self, input_data):
        """
        Setter for Star Tracker data.
        Is used by dbus.
        """

        self._lock.acquire()
        self._st_data = input_data
        self._lock.release()


    # ------------------------------------------------------------------------
    # Magnetometers


    @property
    def MagnetometersData(self):
        """
        Getter for Magnetometers data.
        Is used by dbus and can be use by other classes.
        """

        self._lock.acquire()
        temp = self._magnetometers_data
        self._lock.release()

        return temp


    @MagnetometersData.setter
    def MagnetometersData(self, input_data):
        """
        Setter for Magnetometers data.
        Is used by dbus.
        """

        self._lock.acquire()
        self._magnetometers_data = input_data
        self._lock.release()


    # ------------------------------------------------------------------------
    # ReactionWheels


    @property
    def ReactionWheelsData(self):
        """
        Getter for Reaction Wheels data.
        Is used by dbus and can be use by other classes.
        """

        self._lock.acquire()
        temp = self._reaction_wheels_data
        self._lock.release()

        return temp


    @ReactionWheelsData.setter
    def ReactionWheelsData(self, input_data):
        """
        Setter for Reaction Wheels data.
        Is used by dbus.
        """

        self._lock.acquire()
        self._reaction_wheels_data = input_data
        self._lock.release()


    # ------------------------------------------------------------------------
    # Magnetorquer


    @property
    def MagnetorquerData(self):
        """
        Getter for Magnetorquer data.
        Is used by dbus and can be use by other classes.
        """

        self._lock.acquire()
        temp = self._magnetorquer_data
        self._lock.release()

        return temp


    @MagnetorquerData.setter
    def MagnetorquerData(self, input_data):
        """
        Setter for Magnetorquer data.
        Is used by dbus.
        """

        self._lock.acquire()
        self._magnetorquer_data = input_data
        self._lock.release()

    # ------------------------------------------------------------------------
    # convenience call for all (data not for dbus)

    def get_all_adcs_data(self):
        self._lock.acquire()
        temp = (self._gps_data,
                self._st_data,
                self._magnetometers_data,
                self._reaction_wheels_data,
                self._magnetorquer_data)
        self._lock.release()
        return temp

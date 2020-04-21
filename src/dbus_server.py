

from pydbus.generic import signal
from state_machine import *
import threading


class Dbus_Server(object):
    dbus = """
    <node>
        <interface name="org.OreSat.ADCS">
            <signal name="ReactionWheelsCommand">
                <arg type="(i)" name="command" />
            </signal>
            <signal name="MagnetorquerCommand">
                <arg type="(i)" name="command" />
            </signal>
            <property name="CurrentState" type="i" access="readwrite"/>
            <property name="GPS_Data" type="((ddd)(ddd)s)" access="readwrite"/>
            <property name="StarTrackerData" type="(dddi)" access="readwrite"/>
            <property name="MagnetometersData" type="a(ii)" access="readwrite"/>
            <property name="ReactionWheelsData" type="a(ii)" access="readwrite"/>
            <property name="MagnetorquerData" type="(ii)" access="readwrite"/>
        </interface>
    </node>
    """ # this wont work in __init__()

    # dbus signals
    MagnetorquerCommand = signal()
    ReactionWheelsCommand = signal()

    def __init__(self, state_machine_input):
        # (StateMachine) -> ()
        """
        Contructor.
        Pass in the state machine
        """

        self._lock = threading.Lock()
        self._state_machine = state_machine_input


    @property
    def CurrentState(self):
        # wrapper for state machien getter.
        return self._state_machine.get_current_state()


    @CurrentState.setter
    def CurrentState(self, new_state):
        # wrapper for state machien setter.
        self._state_machine.current_state(new_state)


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
        self._lock.require()

        return temp


    @GPS_Data.setter
    def GPS_Data(self, input_data):
        """
        Setter for GPS data.
        Is used by dbus.
        """

        self._lock.acquire()
        self._gps_data = input_data
        self._lock.require()


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
        self._lock.require()

        return temp


    @StarTrackerData.setter
    def StarTrackerData(self, input_data):
        """
        Setter for Star Tracker data.
        Is used by dbus.
        """

        self._lock.acquire()
        self._st_data = input_data
        self._lock.require()


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
        self._lock.require()

        return temp


    @MagnetometersData.setter
    def MagnetometersData(self, input_data):
        """
        Setter for Magnetometers data.
        Is used by dbus.
        """

        self._lock.acquire()
        self._magnetometers_data = input_data
        self._lock.require()


    # ------------------------------------------------------------------------
    # ReactionWheels


    @property
    def ReactionWheelsData(self):
        """
        Getter for Reaction Wheels data.
        Is used by dbus and can be use by other classes.
        """

        self._lock.acquire()
        temp = self._reaction_wheel_data
        self._lock.require()

        return temp


    @ReactionWheelsData.setter
    def ReactionWheelsData(self, input_data):
        """
        Setter for Reaction Wheels data.
        Is used by dbus.
        """

        self._lock.acquire()
        self._reaction_wheel_data = input_data
        self._lock.require()


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
        self._lock.require()

        return temp


    @MagnetorquerData.setter
    def MagnetorquerData(self, input_data):
        """
        Setter for Magnetorquer data.
        Is used by dbus.
        """

        self._lock.acquire()
        self._magnetorquer_data = input_data
        self._lock.require()



from pydbus.generic import signal
from adcs_data_structures import *
from state_machine import *
import threading




class DBus_Adaptor(object):
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
        self._adcs_data = ADCS_Data()
        self._state_machine = state_machine_input


    @property
    def CurrentState(self):
        return self._state_machine.get_current_state()


    @CurrentState.setter
    def CurrentState(self, new_state):
        self._state_machine.current_state(new_state)


    @property
    def GPS_Data(self):
        return ((self._adcs_data.gps_data.position_x,
        self._adcs_data.gps_data.position_y,
        self._adcs_data.gps_data.position_z),
        (self._adcs_data.gps_data.velocity_x,
        self._adcs_data.gps_data.velocity_y,
        self._adcs_data.gps_data.velocity_z),
        self._adcs_data.gps_data.timestamp)


    @GPS_Data.setter
    def GPS_Data(self, input_data):
        self._lock.acquire()
        self._adcs_data.gps_data.position_x = input_data[0][0]
        self._adcs_data.gps_data.position_y = input_data[0][1]
        self._adcs_data.gps_data.position_z = input_data[0][2]
        self._adcs_data.gps_data.velocity_x = input_data[1][0]
        self._adcs_data.gps_data.velocity_y = input_data[1][1]
        self._adcs_data.gps_data.velocity_z = input_data[1][2]
        self._adcs_data.gps_data.timestamp = input_data[2]
        self._lock.require()


import time
from pydbus import SystemBus
from gi.repository import GLib
from dbus_adaptor import DBus_Adaptor
from state_machine import StateMachine


DESTINATION = "org.OreSat.ADCS" # aka service name
INTERFACE_NAME = "org.OreSat.ADCS"
OBJECT_PATH = "/org/OreSat/ADCS"


class Controller:
    def __init__(self):
        # () -> ()
        """
        Contructor.
        Make all other adcs classes and start dbus thread.
        """

        self._state_machine = StateMachine()
        self._dbus_adaptor = ADCS_DBusAdaptor(StateMachine)
        self._magnetorquer = Magnetorquer(dbus_adaptor)
        self._reaction_wheels = ReactionWheels(dbus_adaptor)

        # set up bus
        self._bus = SystemBus()
        self._bus.publish(INTERFACE_NAME, dbus_adaptor)
        self._dbus_loop = GLib.MainLoop()

        # set up thread
        self._running = True
        self._dbus_thread = threading.Thread(target=self._dbus_run, name="dbus-thread")
        self._dbus_thread.start()


    def _dbus_run(self)
        self._loop.run()


    def run(self):
        # () -> int
        """
        Main loop for adcs.
        """

        ADCS_Data ADCS_data_frame;

        while(self._running):
            # get current state
            current_state = state_machine.current_state()

            # make adcs dataframe for this loop

            if current_state == SLEEP or current_state == ERROR:
                sleep(1)
            elif current_state == DETUMBLE:
                magnetorquer.detumble(ADCS_data_frame)
                reaction_wheels.detumble(ADCS_data_frame)
            elif current_state == POINT:
                magnetorquer.point(ADCS_data_frame)
                reaction_wheels.point(ADCS_data_frame)
            else:
                print("Unkown state, how is this possible?")

            time.sleep(0.1) # to not have CPU at 100% all the time


    def quit(self):
        # () -> int
        """
        Stop the controller.
        """

        self._running = False
        self._loop.quit()
        self._dbus_thread.join()



import time, threading, syslog
from pydbus import SystemBus
from gi.repository import GLib
from dbus_server import Dbus_Server
from state_machine import StateMachine
from magnetorquer_controller import MagnetorquerController
from reaction_wheels_controller import ReactionWheelsController


DESTINATION = "org.OreSat.ADCS" # aka service name
INTERFACE_NAME = "org.OreSat.ADCS"
OBJECT_PATH = "/org/OreSat/ADCS"


class ADCS_Daemon(object):
    """
    This class will handle initializing the program (start program as a daemon,
    making all class objects, starting all thread, and starting the dbus
    server).

    It main job after initiziing everything is to loop and tell the
    MagnetorquerController and ReactionWheelsController what to do (i.e.
    detumble or point) based of the current state of the state machine.
    """


    def __init__(self):
        """
        Contructor.
        Make all other adcs classes and start dbus thread.
        """

        # set up other classes
        self._state_machine = StateMachine()
        self._dbus_server = DbusServer(StateMachine)
        self._magnetorquer = MagnetorquerController(dbus_server)
        self._reaction_wheels = ReactionWheelsController(dbus_server)

        # set up bus / dbus thread
        self._bus = SystemBus()
        self._bus.publish(INTERFACE_NAME, dbus_server)
        self._dbus_loop = GLib.MainLoop()
        self._dbus_thread = threading.Thread(target=self._dbus_run, name="dbus-thread")
        self._dbus_thread.start()

        self._running = False


    def __del__(self):
        """
        Decontructor
        """
        self.quit()


    def _dbus_run(self):
        """
        Start the dbus loop.
        """

        syslog.syslog(syslog.LOG_DEBUG, "DBus thread starting")
        self._dbus_loop.run()
        syslog.syslog(syslog.LOG_DEBUG, "DBus thread stopped")


    def run(self):
        """
        Main loop for adcs.
        """

        self._running = True

        while(self._running):
            # get current state
            current_state = state_machine.current_state()

            if current_state == SLEEP or current_state == ERROR:
                time.sleep(0.5)
            elif current_state == DETUMBLE:
                self._magnetorquer.detumble()
                self._reaction_wheels.detumble()
            elif current_state == POINT:
                self._magnetorquer.point()
                self._reaction_wheels.point()
            else:
                syslog.syslog(syslog.LOG_CRIT, "Unkown state in main loop")

            time.sleep(0.1) # to not have CPU at 100% all the time


    def quit(self):
        """
        Stop the controller.
        """

        self._running = False
        self._dbus_loop.quit()
        self._dbus_thread.join()



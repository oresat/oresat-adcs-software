import time, threading, syslog
from pydbus import SystemBus
from gi.repository import GLib
from dbus_server import DbusServer
from state_machine import *
from magnetorquer_controller import MagnetorquerController
from reaction_wheels_controller import ReactionWheelsController


DESTINATION = "org.OreSat.ADCS" # aka service name
INTERFACE_NAME = "org.OreSat.ADCS"
OBJECT_PATH = "/org/OreSat/ADCS"


class ADCS_Daemon(object):
    """
    This class will handle initializing the program (making all class objects,
    starting all thread, and starting the dbus server).

    NOTE:
        This will NOT make the program into daemon (forking the process) that
        should be done in main().

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
        self._dbus_server = DbusServer(self._state_machine)
        self._magnetorquer = MagnetorquerController()
        self._reaction_wheels = ReactionWheelsController()

        # set up bus / dbus thread
        self._bus = SystemBus()
        self._bus.publish(INTERFACE_NAME, self._dbus_server)
        self._dbus_loop = GLib.MainLoop()
        self._dbus_thread = threading.Thread(target=self._dbus_run, name="dbus-thread")
        self._dbus_thread.start()

        self._running = False


    def __del__(self):
        """
        Decontructor
        Stop dbus thread and dbus server.
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
            current_state = self._state_machine.get_current_state()

            if current_state == State.SLEEP.value or current_state == State.FAILED.value:
                time.sleep(0.5)

            elif current_state == State.DETUMBLE.value:
                # update dataframe
                adcs_data_frame = self._dbus_server.get_all_adcs_data()

                # calcualte and send magnetorquer command
                mag_command = self._magnetorquer.detumble(adcs_data_frame)
                self._dbus_server.MagnetorquerCommand(mag_command[0], mag_command[1])

                # calculate and send rection wheel command
                rw_command = self._reaction_wheels.detumble(adcs_data_frame)
                self._dbus_server.ReactionWheelsCommand(rw_command[0], rw_command[1])

            elif current_state == State.POINT.value:
                # update dataframe
                adcs_data_frame = self._dbus_server.get_all_adcs_data()

                # calcualte and send magnetorquer command
                mag_command = self._magnetorquer.point(adcs_data_frame)
                self._dbus_server.MagnetorquerCommand(mag_command[0], mag_command[1])

                # calculate and send rection wheel command
                rw_command = self._reaction_wheels.point(adcs_data_frame)
                self._dbus_server.ReactionWheelsCommand(rw_command[0], rw_command[1])

            else:
                syslog.syslog(syslog.LOG_CRIT, "Unkown state in main loop")

            time.sleep(0.5) # to not have CPU at 100% all the time


    def quit(self):
        """
        Stop the controller.
        """

        self._running = False
        self._dbus_loop.quit()
        self._dbus_thread.join()



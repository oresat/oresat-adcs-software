from pydbus import SystemBus
from gi.repository import GLib
import time
from threading import Thread
from syslog import syslog
from state_machine import *
from dbus_server import DbusServer
#from ..adcs_lib import controller
import copy


DESTINATION =       "org.OreSat.ADCSManager"
INTERFACE_NAME =    "org.OreSat.ADCSManager"
OBJECT_PATH =       "/org/OreSat/ADCSManager"
PID_FILE =          "/run/oresat-linux-updater.pid"


class ADCSManager():
    """
    This class will handle initializing the program (making all class objects
    and starting all thread).
    """

    # controllers
    #mag_controller = controller.MagnetorquerController()
    #rw_controller = controller.ReactionWheelsController(None, 0, 0)

    def __init__(self):
        """
        Make pid file / check if daemon is already running.
        """

        self._dbus_thread = Thread(target=self._dbus_server_thread)
        self._running = False
        self._dbus_server = DbusServer()
        self._bus = SystemBus()
        self._bus.publish(INTERFACE_NAME, self._dbus_server)
        self._dbus_loop = GLib.MainLoop()

        # Check for a pidfile to see if the daemon is already running
        #try:
        #    with open(PID_FILE,'r') as pf:

#                pid = int(pf.read().strip())
#        except IOError:
#            pid = None

#        if pid:
#            sys.stderr.write("pid file {0} already exist.\n".format(PID_FILE))
#            sys.exit(1)

#        try:
#            pid = os.fork()
#            if pid > 0:
#                # exit from parent
#                sys.exit(0)
#        except OSError as err:
#            sys.stderr.write('fork failed: {0}\n'.format(err))
#            sys.exit(1)

        # decouple from parent environment
#        os.chdir('/')
#        os.setsid()
#        os.umask(0)

        # redirect standard file descriptors
#        sys.stdout.flush()
#        sys.stderr.flush()
#        si = open(os.devnull, 'r')
#        so = open(os.devnull, 'a+')
#        se = open(os.devnull, 'a+')

#        os.dup2(si.fileno(), sys.stdin.fileno())
#        os.dup2(so.fileno(), sys.stdout.fileno())
#        os.dup2(se.fileno(), sys.stderr.fileno())

        # make pid file
#        pid = str(os.getpid())
#        with open(PID_FILE,'w+') as f:
#            f.write(pid + '\n')


    def __del__(self):
        """
        Decontructor to make sure the dbus thread is stopped and to remove
        the pid file.
        """

        # join the thread
        self._dbus_thread.join()

        # remove pid file
#        os.remove(PID_FILE)


    def _dbus_server_thread(self):
        """
        Nice function to hold the dbus thread.
        """

        self._dbus_loop.run()


    def run(self):
        """
        Main loop for adcs.
        """

        # start dbus thread
        self._dbus_thread.start()

        self._running = True
        while(self._running):

            # copy all data from the dbus
            self._dbus_server._data_lock.acquire()
            dbus_data = copy.deepcopy(self._dbus_server._board_data)
            self._dbus_server._data_lock.release()

            # send data to 42
            self._dbus_server.VisualizationDataSignal(
                [0.0, 0.0, 0.0],                # sun-pointing unit vector
                dbus_data.mag_field_vector0,    # magnetic field vector
                [0.0, 0.0, 0.0],                # angular momentum
                dbus_data.angular_velocity,     # angular velocity
                [0.0, 0.0, 0.0, 0.0],           # spacecraft attitude
                [0.0, 0.0, 0.0],                # central orbit position
                [0.0, 0.0, 0.0]                 # central orbit velocity
            )
            
            # TODO: integrate these commands with state machine

            # magnetorquer command
            #mag_x_command = self.mag_controller.detumble(None, 0)
            #mag_y_command = self.mag_controller.detumble(None, 0)
            #mag_z_command = self.mag_controller.detumble(None, 0)
            mag_x_command = 0
            mag_y_command = 0
            mag_z_command = 0
            self._dbus_server.MagnetorquerCommand(mag_x_command, mag_y_command, mag_z_command)
            self._dbus_server.NewMagCommands(mag_x_command, mag_y_command, mag_z_command)

            # reaction wheels command, call adcs lib functions here eventually
            rw1_command = 0
            rw2_command = 0
            rw3_command = 0
            rw4_command = 0
            self._dbus_server.ReactionWheelsCommand(rw1_command, rw2_command, rw3_command, rw4_command)
            self._dbus_server.NewRWCommands(rw1_command, rw2_command, rw3_command, rw4_command)

            current_state = self._dbus_server.CurrentMode

            if current_state == State.SLEEP.value or current_state == State.FAILED.value:
                time.sleep(0.5)
            elif current_state == State.DETUMBLE.value:
                temp = 1 # TODO call manager fuction
            elif current_state == State.POINT.value:
                temp = 1 # TODO call manager fuction
            else:
                syslog(LOG_CRIT, "Unkown state in main loop")

            time.sleep(0.5) # so CPU is not at 100% all the time


    def quit(self):
        """
        Stop all thread.
        """

        self._running = False
        self._dbus_loop.quit()

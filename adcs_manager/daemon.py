from pydbus import SystemBus
from gi.repository import GLib
from time import sleep
from threading import Thread
from syslog import syslog
from state_machine import *


DESTINATION =       "org.OreSat.ADCSManager"
INTERFACE_NAME =    "org.OreSat.ADCSManager"
OBJECT_PATH =       "/org/OreSat/ADCSManager"
PID_FILE =          "/run/oresat-linux-updater.pid"


class Daemon():
    """
    This class will handle initializing the program (making all class objects
    and starting all thread).
    """

    _dbus_thread = Thread(target=self._dbus_server_thread)
    _running = False
    _bus = SystemBus()
    _bus.publish(INTERFACE_NAME, self._dbus_server)
    _dbus_loop = GLib.MainLoop()


    def __init__(self):
        """
        Make pid file / check if daemon is already running.
        """

        # Check for a pidfile to see if the daemon is already running
        try:
            with open(PID_FILE,'r') as pf:

                pid = int(pf.read().strip())
        except IOError:
            pid = None

        if pid:
            sys.stderr.write("pid file {0} already exist.\n".format(PID_FILE))
            sys.exit(1)

        try:
            pid = os.fork()
            if pid > 0:
                # exit from parent
                sys.exit(0)
        except OSError as err:
            sys.stderr.write('fork failed: {0}\n'.format(err))
            sys.exit(1)

        # decouple from parent environment
        os.chdir('/')
        os.setsid()
        os.umask(0)

        # redirect standard file descriptors
        sys.stdout.flush()
        sys.stderr.flush()
        si = open(os.devnull, 'r')
        so = open(os.devnull, 'a+')
        se = open(os.devnull, 'a+')

        os.dup2(si.fileno(), sys.stdin.fileno())
        os.dup2(so.fileno(), sys.stdout.fileno())
        os.dup2(se.fileno(), sys.stderr.fileno())

        # make pid file
        pid = str(os.getpid())
        with open(PID_FILE,'w+') as f:
            f.write(pid + '\n')


    def __del__(self):
        """
        Decontructor to make sure the dbus thread is stopped and to remove
        the pid file.
        """

        # join the thread
        self._dbus_thread.join()

        # remove pid file
        os.remove(PID_FILE)


    def _dbus_server_thead(self):
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
            # TODO check for new mode & swap to new mode if set

            current_state = self._dbus_server.CurrentMode()

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

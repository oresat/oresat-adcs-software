from pydbus import SystemBus
from gi.repository import GLib
import time
from threading import Thread
from syslog import syslog
from adcs_lib.state_machine import *
from dbus_server import DbusServer
from adcs_lib import manager_class, simulator, quaternion
import copy
import numpy as np

T_SLEEP  = 0.5
T_CTRL   = 0.05
DEBUG    = True

DESTINATION    =    "org.OreSat.ADCSManager"
INTERFACE_NAME =    "org.OreSat.ADCSManager"
OBJECT_PATH    =    "/org/OreSat/ADCSManager"
PID_FILE       =    "/run/oresat-linux-updater.pid"


class ADCSManager():
    """
    This class will handle initializing the program (making all class objects
    and starting all thread).
    """

    # adcs library interface
    sim = simulator.SimulatorDaemonInterface()
    man = manager_class.ManagerDaemonInterface(T_CTRL, T_CTRL, sim.model)

    def __init__(self):
        """
        Make pid file / check if daemon is already running.
        """

        self._dbus_thread = Thread(target=self._dbus_server_thread)
        self._running     = False
        self._dbus_server = DbusServer()
        self._bus         = SystemBus()
        self._bus.publish(INTERFACE_NAME, self._dbus_server)
        self._dbus_loop   = GLib.MainLoop()

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

        # for debugging
        if DEBUG:
            self.true_states   = []
            self.kalman_states = []

        # start dbus thread
        self._dbus_thread.start()

        self._dbus_server._sm.change_state(State.SLEEP.value)

        self._running = True
        counter       = 0 # for testing out periodic sensor measurements

        while(self._running):

            # copy all data from the dbus
            self._dbus_server._data_lock.acquire()
            dbus_data = copy.deepcopy(self._dbus_server._board_data)
            self._dbus_server._data_lock.release()

            # make adcs library calls
            current_state = self._dbus_server.CurrentMode
            cmds          = None
            sim_output    = None

            # for testing out periodic sensor measurements
            raw_sensor_data = None if counter != 0 else self.sim.output(noisy=True)
            counter         = (counter + 1) % 1200

            if current_state == State.SLEEP.value or current_state == State.FAILED.value:
                cmds       = self.man.propagate(T_SLEEP + T_CTRL, [current_state, np.zeros(3), np.zeros(3), np.zeros(3)], raw_sensor_data)
                sim_output = self.sim.propagate(T_SLEEP + T_CTRL, cmds[:2])
                if not DEBUG:
                    time.sleep(T_SLEEP)

            elif current_state == State.DETUMBLE.value:
                cmds       = self.man.propagate(T_CTRL, [State.DETUMBLE.value, np.zeros(3), np.zeros(3), np.zeros(3)], raw_sensor_data)
                sim_output = self.sim.propagate(T_CTRL, cmds[:2])

            elif current_state == State.POINT.value:
                cmds       = self.man.propagate(T_CTRL, [State.POINT.value, np.array([0, 0, -1]), np.zeros(3), np.zeros(3)], raw_sensor_data)
                sim_output = self.sim.propagate(T_CTRL, cmds[:2])
            elif current_state == State.BBQ.value:
                cmds       = self.man.propagate(T_CTRL, [State.BBQ.value, np.zeros(3), np.zeros(3), np.zeros(3)], raw_sensor_data)
                sim_output = self.sim.propagate(T_CTRL, cmds[:2])

            else:
                syslog(LOG_CRIT, "Unknown state in main loop")

            [mag_commands, rw_commands] = cmds[:2]
            (x, v, q, w, W, S, B)       = sim_output

            if DEBUG:
                q_d = self.man.mag_controller.ecliptic_attitude(True, q)
                self.true_states.append(np.block([x, v, q, w, W, B, S, self.man.filter.PosFilter.model.satellite.magnetorquers.power(mag_commands), np.sum([* np.abs(mag_commands)]), q_d]))
                self.kalman_states.append(np.block([*cmds[-1], *quaternion.error_quat(q, cmds[-1][2])[1:]*0.5])) # to look at error between state and estimate
                #self.kalman_states.append(np.block([*cmds[-1], *quaternion.error_quat(q, q_d)[1:] * 0.5])) # to look at error between state and command

            if DEBUG:
                # need to figure out a systematic way to transition between states
                if current_state == State.DETUMBLE.value and np.linalg.norm(w) < 0.0075:
                    print('done detumbling')
                    self._dbus_server._sm.change_state(State.BBQ.value)
                #if current_state == State.BBQ.value and self.man.mag_controller.converged:
                    #print('done bbqing')
                    #self._dbus_server._sm.change_state(State.SLEEP.value)
                if self.man.filter.PosFilter.model.clock.absolute > 5560*4:
                    self.quit()


            # send data to 42
            self._dbus_server.VisualizationDataSignal(
                S,                                   # sun-pointing unit vector
                B,                                   # magnetic field vector
                self.man.rw_controller.MoI.dot(w),   # angular momentum
                w,                                   # angular velocity
                np.array([*q[1:], q[0]]),            # spacecraft attitude
                x,                                   # central orbit position
                v                                    # central orbit velocity
            )

            # magnetorquer command
            self._dbus_server.MagnetorquerCommand(*mag_commands)
            self._dbus_server.NewMagCommands(*mag_commands)

            # reaction wheels command, call adcs lib functions here eventually
            self._dbus_server.ReactionWheelsCommand(*rw_commands)
            self._dbus_server.NewRWCommands(*rw_commands)

            if not DEBUG:
                time.sleep(T_CTRL) # so CPU is not at 100% all the time


    def quit(self):
        """
        Stop all thread.
        """
        if DEBUG:
            # for debugging
            np.savetxt('true_states.csv', self.true_states, delimiter=',', comments='',
                header='x_x,x_y,x_z,v_x,v_y,v_z,q0,q1,q2,q3,w_x,w_y,w_z,W1,W2,W3,W4,B_x,B_y,B_z,S_x,S_y,S_z,P,I,qd0,qd1,qd2,qd3')
            np.savetxt('kalman_states.csv', self.kalman_states, delimiter=',', comments='',
                header='x_x,x_y,x_z,v_x,v_y,v_z,q0,q1,q2,q3,w_x,w_y,w_z,W1,W2,W3,W4,B_x,B_y,B_z,S_x,S_y,S_z,roll,pitch,yaw')

        self._running = False
        self._dbus_loop.quit()

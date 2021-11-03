from enum import Enum
from threading import Lock
import numpy as np

class State(Enum):
    FAILED   = 0      # Something failed.
    SLEEP    = 1       # ADCS program is not doing anything, most subsystems off
    UNSAFE   = 2  # It is not safe to shut ADCS card off
    SAFE     = 3 # Waiting for commands, and it is safe to shut ADCS card off
    SPINDOWN = 4 # Spinning wheels down and then turning them off
    DETUMBLE = 5    # Detumble the satellite
    BBQ      = 6    # Initiate BBQ roll
    POINT    = 7       # Point the satellite for CFC / Oresat Live
    MANUAL   = 8  # Give commands manually

    @classmethod
    def transition(cls, state, man, mission, position, lin_vel, attitude, body_ang_vel, wheel_vel, mag_field, cur_cmd, whl_accl):
        '''Dispatch method'''
        method_name = state.name + '_transition'
        method = getattr(cls, method_name)
        return method(cls, man, mission, position, lin_vel, attitude, body_ang_vel, wheel_vel, mag_field, cur_cmd, whl_accl)


    def SAFE_transition(self, man, mission, position, lin_vel, attitude, body_ang_vel, wheel_vel, mag_field, cur_cmd, whl_accl):
        '''
        Checks whether the current status of the satellite meets the transition conditions for SAFE.
        If so, performs the appropriate transition.

        Parameters
        ----------
        man : ManagerDaemonInterface
            Manager instance (currently kind of a hack)
        mission : Mission
            Next mission.
        position : numpy.ndarray
            Present position in ECI coordinates
        lin_vel : numpy.ndarray
            Present velocity in ECI coordinates
        attitude : numpy.ndarray
            Present attitude quaternion with respect to inertial frame.
        body_ang_vel : numpy.ndarray
            Present angular velocity in body frame coordinates with respect to inertial frame.
        wheel_vel : numpy.ndarray
            Present velocities of reaction wheels.
        mag_field : numpy.ndarray
            Present magnetic field reading.
        cur_cmd : numpy.ndarray
            Magnetorquer commands for currents.
        whl_accl : numpy.ndarray
            Reaction wheel acceleration commands.
        '''

        mission_now = man.filter.PosFilter.model.clock.is_after(mission.start)

        if mission_now:
            return State.POINT
        else:
            return State.SAFE

    def DETUMBLE_transition(self, man, mission, position, lin_vel, attitude, body_ang_vel, wheel_vel, mag_field, cur_cmd, whl_accl):
        '''
        Checks whether the current status of the satellite meets the transition conditions for DETUMBLE.
        If so, performs the appropriate transition.

        Parameters
        ----------
        man : ManagerDaemonInterface
            Manager instance (currently kind of a hack)
        mission : Mission
            Next mission.
        position : numpy.ndarray
            Present position in ECI coordinates
        lin_vel : numpy.ndarray
            Present velocity in ECI coordinates
        attitude : numpy.ndarray
            Present attitude quaternion with respect to inertial frame.
        body_ang_vel : numpy.ndarray
            Present angular velocity in body frame coordinates with respect to inertial frame.
        wheel_vel : numpy.ndarray
            Present velocities of reaction wheels.
        mag_field : numpy.ndarray
            Present magnetic field reading.
        cur_cmd : numpy.ndarray
            Magnetorquer commands for currents.
        whl_accl : numpy.ndarray
            Reaction wheel acceleration commands.
        '''

        interrupt = self.SAFE_transition(man, mission, position, lin_vel, attitude, body_ang_vel, wheel_vel, mag_field, cur_cmd, whl_accl)
        if interrupt is State.POINT:
            return State.POINT

        too_fast = np.linalg.norm(body_ang_vel) >= 0.05
        if too_fast:
            return State.DETUMBLE
        else:
            return State.BBQ

    def BBQ_transition(self, man, mission, position, lin_vel, attitude, body_ang_vel, wheel_vel, mag_field, cur_cmd, whl_accl):
        '''
        Checks whether the current status of the satellite meets the transition conditions for BBQ.
        If so, performs the appropriate transition.

        Parameters
        ----------
        man : ManagerDaemonInterface
            Manager instance (currently kind of a hack)
        mission : Mission
            Next mission.
        position : numpy.ndarray
            Present position in ECI coordinates
        lin_vel : numpy.ndarray
            Present velocity in ECI coordinates
        attitude : numpy.ndarray
            Present attitude quaternion with respect to inertial frame.
        body_ang_vel : numpy.ndarray
            Present angular velocity in body frame coordinates with respect to inertial frame.
        wheel_vel : numpy.ndarray
            Present velocities of reaction wheels.
        mag_field : numpy.ndarray
            Present magnetic field reading.
        cur_cmd : numpy.ndarray
            Magnetorquer commands for currents.
        whl_accl : numpy.ndarray
            Reaction wheel acceleration commands.
        '''

        interrupt = self.SAFE_transition(man, mission, position, lin_vel, attitude, body_ang_vel, wheel_vel, mag_field, cur_cmd, whl_accl)
        if interrupt is State.POINT:
            return State.POINT

        aligned  = man.mag_controller.check(attitude, body_ang_vel) < 1e-4
        low_ctrl = np.linalg.norm(cur_cmd) < 1e-4
        low_whls = np.linalg.norm(man.rw_controller.satellite.reaction_wheels.axes.dot(wheel_vel)) < 30
        if aligned and low_ctrl and low_whls:
            return State.SAFE
        else:
            return State.BBQ

    def UNSAFE_transition(self, man, mission, position, lin_vel, attitude, body_ang_vel, wheel_vel, mag_field, cur_cmd, whl_accl):
        '''
        Checks whether the current status of the satellite meets the transition conditions for UNSAFE.
        If so, performs the appropriate transition.

        Parameters
        ----------
        man : ManagerDaemonInterface
            Manager instance (currently kind of a hack)
        mission : Mission
            Next mission.
        position : numpy.ndarray
            Present position in ECI coordinates
        lin_vel : numpy.ndarray
            Present velocity in ECI coordinates
        attitude : numpy.ndarray
            Present attitude quaternion with respect to inertial frame.
        body_ang_vel : numpy.ndarray
            Present angular velocity in body frame coordinates with respect to inertial frame.
        wheel_vel : numpy.ndarray
            Present velocities of reaction wheels.
        mag_field : numpy.ndarray
            Present magnetic field reading.
        cur_cmd : numpy.ndarray
            Magnetorquer commands for currents.
        whl_accl : numpy.ndarray
            Reaction wheel acceleration commands.
        '''

        interrupt = self.SAFE_transition(man, mission, position, lin_vel, attitude, body_ang_vel, wheel_vel, mag_field, cur_cmd, whl_accl)
        if interrupt is State.POINT:
            return State.POINT

        state = self.DETUMBLE_transition(man, position, lin_vel, attitude, body_ang_vel, wheel_vel, mag_field, cur_cmd, whl_accl)
        if state is State.BBQ:
            state = self.BBQ_transition(man, position, lin_vel, attitude, body_ang_vel, wheel_vel, mag_field, cur_cmd, whl_accl)
        return state

    def SPINDOWN_transition(self, man, mission, position, lin_vel, attitude, body_ang_vel, wheel_vel, mag_field, cur_cmd, whl_accl):
        '''
        Checks whether the current status of the satellite meets the transition conditions for SPINDOWN.
        If so, performs the appropriate transition.

        Parameters
        ----------
        man : ManagerDaemonInterface
            Manager instance (currently kind of a hack)
        mission : Mission
            Next mission.
        position : numpy.ndarray
            Present position in ECI coordinates
        lin_vel : numpy.ndarray
            Present velocity in ECI coordinates
        attitude : numpy.ndarray
            Present attitude quaternion with respect to inertial frame.
        body_ang_vel : numpy.ndarray
            Present angular velocity in body frame coordinates with respect to inertial frame.
        wheel_vel : numpy.ndarray
            Present velocities of reaction wheels.
        mag_field : numpy.ndarray
            Present magnetic field reading.
        cur_cmd : numpy.ndarray
            Magnetorquer commands for currents.
        whl_accl : numpy.ndarray
            Reaction wheel acceleration commands.
        '''
        low_accl = np.linalg.norm(whl_accl) < 0.00001
        low_vel  = np.linalg.norm(wheel_vel) < 0.01
        if low_accl and low_vel:
            return State.SLEEP
        else:
            return State.SPINDOWN

    def FAILED_transition(self, man, mission, position, lin_vel, attitude, body_ang_vel, wheel_vel, mag_field, cur_cmd, whl_accl):
        '''
        Checks whether the current status of the satellite meets the transition conditions for FAILED.
        If so, performs the appropriate transition.

        Parameters
        ----------
        man : ManagerDaemonInterface
            Manager instance (currently kind of a hack)
        mission : Mission
            Next mission.
        position : numpy.ndarray
            Present position in ECI coordinates
        lin_vel : numpy.ndarray
            Present velocity in ECI coordinates
        attitude : numpy.ndarray
            Present attitude quaternion with respect to inertial frame.
        body_ang_vel : numpy.ndarray
            Present angular velocity in body frame coordinates with respect to inertial frame.
        wheel_vel : numpy.ndarray
            Present velocities of reaction wheels.
        mag_field : numpy.ndarray
            Present magnetic field reading.
        cur_cmd : numpy.ndarray
            Magnetorquer commands for currents.
        whl_accl : numpy.ndarray
            Reaction wheel acceleration commands.
        '''

        if False:
            return State.UNSAFE
        else:
            return State.FAILED

    def SLEEP_transition(self, man, mission, position, lin_vel, attitude, body_ang_vel, wheel_vel, mag_field, cur_cmd, whl_accl):
        '''
        Checks whether the current status of the satellite meets the transition conditions for SLEEP.
        If so, performs the appropriate transition.

        Parameters
        ----------
        man : ManagerDaemonInterface
            Manager instance (currently kind of a hack)
        mission : Mission
            Next mission.
        position : numpy.ndarray
            Present position in ECI coordinates
        lin_vel : numpy.ndarray
            Present velocity in ECI coordinates
        attitude : numpy.ndarray
            Present attitude quaternion with respect to inertial frame.
        body_ang_vel : numpy.ndarray
            Present angular velocity in body frame coordinates with respect to inertial frame.
        wheel_vel : numpy.ndarray
            Present velocities of reaction wheels.
        mag_field : numpy.ndarray
            Present magnetic field reading.
        cur_cmd : numpy.ndarray
            Magnetorquer commands for currents.
        whl_accl : numpy.ndarray
            Reaction wheel acceleration commands.
        '''

        if False:
            return State.UNSAFE
        else:
            return State.SLEEP

    def POINT_transition(self, man, mission, position, lin_vel, attitude, body_ang_vel, wheel_vel, mag_field, cur_cmd, whl_accl):
        '''
        Checks whether the current status of the satellite meets the transition conditions for POINT.
        If so, performs the appropriate transition.

        Parameters
        ----------
        man : ManagerDaemonInterface
            Manager instance (currently kind of a hack)
        mission : Mission
            Next mission.
        position : numpy.ndarray
            Present position in ECI coordinates
        lin_vel : numpy.ndarray
            Present velocity in ECI coordinates
        attitude : numpy.ndarray
            Present attitude quaternion with respect to inertial frame.
        body_ang_vel : numpy.ndarray
            Present angular velocity in body frame coordinates with respect to inertial frame.
        wheel_vel : numpy.ndarray
            Present velocities of reaction wheels.
        mag_field : numpy.ndarray
            Present magnetic field reading.
        cur_cmd : numpy.ndarray
            Magnetorquer commands for currents.
        whl_accl : numpy.ndarray
            Reaction wheel acceleration commands.
        '''

        mission_over = man.filter.PosFilter.model.clock.is_after(mission.end)

        if mission_over:
            return State.UNSAFE
        else:
            return State.POINT

    def MANUAL_transition(self, man, mission, position, lin_vel, attitude, body_ang_vel, wheel_vel, mag_field, cur_cmd, whl_accl):
        '''
        Checks whether the current status of the satellite meets the transition conditions for MANUAL.
        If so, performs the appropriate transition.

        Parameters
        ----------
        man : ManagerDaemonInterface
            Manager instance (currently kind of a hack)
        mission : Mission
            Next mission.
        position : numpy.ndarray
            Present position in ECI coordinates
        lin_vel : numpy.ndarray
            Present velocity in ECI coordinates
        attitude : numpy.ndarray
            Present attitude quaternion with respect to inertial frame.
        body_ang_vel : numpy.ndarray
            Present angular velocity in body frame coordinates with respect to inertial frame.
        wheel_vel : numpy.ndarray
            Present velocities of reaction wheels.
        mag_field : numpy.ndarray
            Present magnetic field reading.
        cur_cmd : numpy.ndarray
            Magnetorquer commands for currents.
        whl_accl : numpy.ndarray
            Reaction wheel acceleration commands.
        '''

        if False:
            return State.UNSAFE
        else:
            return State.MANUAL

class Mission():
    def __init__(self, start, end, target, instrument):
        '''
        Basic structure of a mission.

        Parameters
        ----------
        start : list
            List of integers defining mission start, in following order:
            Year, Month, Day, Hour, Minute, Second
        end : list
            List of integers defining mission end, in following order:
            Year, Month, Day, Hour, Minute, Second
        target : numpy.ndarray
            Cartesian coordinates of target position in ECEF frame.
        instrument : int
            ID number of pointing instrument.
        '''

        self.start, self.end, self.target, self.instrument = start, end, target, instrument

class StateMachine:
    def __init__(self):
        self._lock = Lock()
        self._current_state = State.SLEEP
        self._valid_transition = {
                State.FAILED : [State.FAILED, State.UNSAFE],
                State.SLEEP : [State.SLEEP, State.FAILED, State.UNSAFE],
                State.UNSAFE : [State.FAILED, State.UNSAFE, State.SAFE, State.DETUMBLE, State.BBQ, State.POINT, State.MANUAL],
                State.SAFE : [State.FAILED, State.SAFE, State.SPINDOWN, State.POINT, State.BBQ, State.MANUAL],
                State.SPINDOWN : [State.FAILED, State.SLEEP, State.SPINDOWN],
                State.DETUMBLE : [State.FAILED, State.DETUMBLE, State.BBQ, State.POINT, State.MANUAL],
                State.BBQ : [State.FAILED, State.SAFE, State.BBQ, State.POINT.value, State.MANUAL],
                State.POINT : [State.FAILED, State.POINT, State.UNSAFE, State.MANUAL],
                State.MANUAL : [State.FAILED, State.POINT, State.UNSAFE, State.MANUAL]
            }
        self._next_mission = None
        self.state_class = State(0)
        

    def get_next_mission(self):
        '''
        Gets the next mission or returns None if there is no next mission.

        Returns
        -------
        Mission or None
        '''
        return self._next_mission

    def set_next_mission(self, start, end=None, target=None, instrument=None):
        '''
        Sets the next mission. Call with single parameter of None if there is no next mission.

        Parameters
        ----------
        start : list
            List of integers defining mission start, in following order:
            Year, Month, Day, Hour, Minute, Second
        end : list
            List of integers defining mission end, in following order:
            Year, Month, Day, Hour, Minute, Second
        target : numpy.ndarray
            Cartesian coordinates of target position in ECEF frame.
        instrument : int
            ID number of pointing instrument.
        '''
        self._lock.acquire()

        if start is None:
            self._next_mission = None
        else:
            self._next_mission = Mission(start, end, target, instrument)

        self._lock.release()

    def change_state(self, new_state):
        # (State) -> bool
        """
        Check if the state change is valid.
        Returns true if valid and transition happend
        Returns false if errored.
        """

        self._lock.acquire()

        # check if new_state is a valid transition
        if new_state in self._valid_transition[self._current_state]:
            self._current_state = new_state
            valid = True
        else:
            self._current_state = State.FAILED
            valid = False # not a valid transition

        self._lock.release()
        return valid


    def get_current_state(self):
        """Returns the state machine's current state."""
        return self._current_state

    def transition(self, man, position, lin_vel, attitude, body_ang_vel, wheel_vel, mag_field, cur_cmd, whl_accl):
        '''
        Checks whether the current status of the satellite meets the transition conditions for its state.
        If so, performs the transition.

        Parameters
        ----------
        position : numpy.ndarray
            Present position in ECI coordinates
        lin_vel : numpy.ndarray
            Present velocity in ECI coordinates
        attitude : numpy.ndarray
            Present attitude quaternion with respect to inertial frame.
        body_ang_vel : numpy.ndarray
            Present angular velocity in body frame coordinates with respect to inertial frame.
        wheel_vel : numpy.ndarray
            Present velocities of reaction wheels.
        mag_field : numpy.ndarray
            Present magnetic field reading.
        '''
        state      = self.get_current_state()
        mission    = self.get_next_mission()
        next_state = self.state_class.transition(state, man, mission, position, lin_vel, attitude, body_ang_vel, wheel_vel, mag_field, cur_cmd, whl_accl)
        if state is not next_state:
            success = self.change_state(next_state)

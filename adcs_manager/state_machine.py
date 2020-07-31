from enum import Enum
from threading import Lock



class State(Enum):
    FAILED = 0      # Something failed.
    SLEEP = 1       # ADCS program is not doing anything and waiting for commands.
    DETUMBLE = 2    # Detumble the satellite after deployement
    POINT = 3       # Point the satellite for CFC / Oresat Live


class StateMachine:
    def __init__(self):
        self._lock = Lock()
        self._current_state = State.SLEEP.value
        self._valid_transition = {
                State.FAILED.value : [State.FAILED.value, State.SLEEP.value],
                State.SLEEP.value : [State.SLEEP.value, State.DETUMBLE.value, State.POINT.value],
                State.DETUMBLE.value : [State.DETUMBLE.value, State.SLEEP.value, State.FAILED.value],
                State.POINT.value : [State.POINT.value, State.SLEEP.value, State.FAILED.value],
            }


    def change_state(self, new_state):
        # (int) -> bool
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
            self._current_state = State.FAILED.value
            valid = False # not a valid transition

        self._lock.release()
        return valid


    def get_current_state(self):
        return self._current_state

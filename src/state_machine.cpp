#include "state_machine.h"


StateMachine::
StateMachine() : current_state(SLEEP) {}


bool StateMachine::
change_state(const int32_t & new_state) {
    bool valid = false;

    /**
     * since this can be call in mutltiple thread need to lock mutex for the
     * whole function.
     */
    state_mutex.lock();

    switch(current_state) {
        case FAILED:
            if(new_state == FAILED)
                valid = true;
            break;

        case SLEEP:
            if(new_state == SLEEP)
                valid = true;
            break;

        case DETUMBLE:
            if(new_state == DETUMBLE || new_state == SLEEP || new_state == FAILED)
                valid = true;
            break;

        case POINT:
            if(new_state == POINT || new_state == SLEEP || new_state == FAILED)
                valid = true;
            break;

        default :
            cout << "state change failed" << endl; // TODO change to syslog later
    }

    if(valid) // transistion was valid, change the state.
        current_state = new_state;

    state_mutex.unlock();
    return valid;
}


int32_t StateMachine::
get_current_state() {
    return current_state;
}

#include <mutex>


enum states {
    // Something failed.
    Error = 0,
    // ADCS program is not doing anything and waiting for commands.
    Sleep = 1,
    // Detumble the satellite after deployement
    Detumble = 2;
    // Point the satellite for CFC / Oresat Live
    Point = 3,
};


class StateMachine {
    public :
        StateMachine();
        ~StateMachine() = default;
        bool change_state(const int & new_state);
        int32_t get_current_state();
    private :
        int32_t current_state;
        std::mutex state_mutex;
};

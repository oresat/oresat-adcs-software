#include "ADCS_dbus_adaptor.h"
#include "reaction_wheels.h"
#include "magnetorquer.h"
#include "state_machine.h"
#include <thread>

class Controller {
    public:
        Controller();
        ~Controller() = default;
        int run();
    private:
        void start_dbus_thread();

        std::unique_ptr<sdbus::IConnection> connection;
        ADCS_Controller ADCS_controller;
        std::shared_ptr<StateMachine> state_machine>;
        std::shared_ptr<ADCS_dbus_adaptor> dbus;
        Magnetorquer magnetorquer;
        ReactionWheels reaction_wheels;
        std::thread dbus_thread;
};

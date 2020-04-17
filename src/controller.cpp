#include "controller.h"


#define DESTINATION     "org.OreSat.ADCS" // aka service name
#define INTERFACE_NAME  "org.OreSat.ADCS"
#define OBJECT_PATH     "/org/OreSat/ADCS"


Controller::
Controller() :
        state_machine(std::make_shared<StateMachine>()),
        dbus(std::make_shared<ADCS_dbus_adaptor>(state_machine)),
        magnetorquer(dbus),
        reaction_wheels(dbus),
        connection(sdbus::createSystemBusConnection(DESTINATION)),
        ADCS_controller(*connection, OBJECT_PATH) {
    dbus_thread(&Controller::start_dbus_thread, this);
}


void Controller::
start_dbus_thread(
    // Run the loop on the connection.
    connection->enterEventLoop();
)


int Controller::
run() {
    int current_state;
    ADCS_Data ADCS_data_frame;

    while(running) {
        // get current state
        current_state = state_machine.current_state();

        // make adcs dataframe for this loop
        ADCS_data_frame = *ADCS_data;

        switch(current_state) {
            case Error:
            case Sleep:
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                break;

            case Detumble:
                magnetorquer.detumble(ADCS_data_frame);
                reaction_wheels.detumble(ADCS_data_frame);
                break;

            case Point:
                magnetorquer.point(ADCS_data_frame);
                reaction_wheels.point(ADCS_data_frame);
                break;

            default :
                std::cout << "Unkown state, how is this possible?" << std::endl;
        }

        // to not have CPU at 100% all the time
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 0;
}

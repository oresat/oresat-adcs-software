#include "reaction_wheels.h"


ReactionWheels::
ReactionWheels(std::shared_ptr<ADCS_dbus_adaptor> input) : dbus(input) {}


int ReactionWheels::
detumble(const ADCS_data & adcs_data) {
    return 0;
}


int ReactionWheels::
point(const ADCS_data & adcs_data) {
    return 0;
}

#include "magetorquer.h"


Magnetorquer::
Magnetorquer(std::shared_ptr<ADCS_dbus_adaptor> input) : dbus(input) {}


int Magnetorquer::
detumble(const ADCS_data & adcs_data) {
    return 0;
}


int Magnetorquer::
point(const ADCS_data & adcs_data) {
    return 0;
}

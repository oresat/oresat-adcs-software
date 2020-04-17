#include "adcs_dbus_server.h"


typedef sdbus::Struct<sdbus::Struct<double, double, double> 3_double_struct;
typedef sdbus::Struct<3_double_struct, 3_double_struct, int32_t> gps_struct;
typedef sdbus::Struct<double, double, double, int32_t> st_struct;


int32_t ADCS_Controller::
CurrentState() override {
    return state_machine.current_state();
}


void ADCS_Controller::
CurrentState(const int32_t& value) override {
    state_machine.current_state(value);
    return;
}


gps_struct ADCS_Controller::
GPS_Data() override {
    gps_struct gps_data = {{1.0, 1.0, 1.0}, {1.0, 1.0, 1.0} 1}
    return gps_data;
}


void ADCS_Controller::
GPS_Data(gps_struct) override {
    //convert sdbus struct into normal struct
    return;
}

st_struct ADCS_Controller::
StarTrackerData() override {
    st_struct st_data = {1.0, 1.0, 1.0, 1}
    return st_data;
}


void ADCS_Controller::
StarTrackerData(const st_struct& value) override {
    //convert sdbus struct into normal struct
    return;
}


std::vector<sdbus::Struct<int32_t, int32_t>> ADCS_Controller::
MagnetometersData() override {
    std::vector<sdbus::Struct<int32_t, int32_t>> magnetometers_data = {1, 1}
    return magnetometers_data;
}


void ADCS_Controller::
MagnetometersData(const std::vector<sdbus::Struct<int32_t, int32_t>>& value) override {
    //convert sdbus struct into normal struct
    return;
}


std::vector<sdbus::Struct<int32_t, int32_t>>
ReactionWheelsData() override {
    std::vector<sdbus::Struct<int32_t, int32_t>> reaction_wheels_data = {1, 1}
    return reaction_wheels_data;
}


void ADCS_Controller::
ReactionWheelsData(const std::vector<sdbus::Struct<int32_t, int32_t>>& value) override {
    //convert sdbus struct into normal struct
    return;
}


sdbus::Struct<int32_t, int32_t> ADCS_Controller::
MagnetorquerData() override {
    std::vector<sdbus::Struct<int32_t, int32_t>> magnetorquer_data = {1, 1}
    return magnetorquer_data;
}


void ADCS_Controller::
MagnetorquerData(const sdbus::Struct<int32_t, int32_t>& value) override {
    //convert sdbus struct into normal struct
    return;
}


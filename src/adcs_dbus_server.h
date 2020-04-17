#ifndef _ADCS_DBUS_SERVER_
#define _ADCS_DBUS_SERVER_

#include <sdbus-c++/sdbus-c++.h>
#include "adcs_dbus_server_glue.h"
#include "StateMachine.h"
#include <mutex>


enum ASCS_states {
    FAILED = 0,
    SLEEP = 1,
    DETUMBLE = 2,
    POINT = 3,
};


class ADCS_Controller : public sdbus::AdaptorInterfaces<org::OreSat::ADCS_adaptor> {
public:
    ADCS_Controller(sdbus::IConnection& connection, std::string objectPath)
        : sdbus::AdaptorInterfaces<org::OreSat::ADCS_adaptor>(connection, std::move(objectPath))
    {
        registerAdaptor();
    }

    ~ADCS_Controller() {
        unregisterAdaptor();
    }

protected:
    int32_t CurrentState() override;

    void CurrentState(const int32_t& value) override;

    sdbus::Struct<sdbus::Struct<double, double, double>, sdbus::Struct<double, double, double>, int32_t> GPS_Data() override;

    void GPS_Data(const sdbus::Struct<sdbus::Struct<double, double, double>, sdbus::Struct<double, double, double>, int32_t>& value) override;

    sdbus::Struct<double, double, double, int32_t> StarTrackerData() override;

    void StarTrackerData(const sdbus::Struct<double, double, double, int32_t>& value) override;

    std::vector<sdbus::Struct<int32_t, int32_t>> MagnetometersData() override;

    void MagnetometersData(const std::vector<sdbus::Struct<int32_t, int32_t>>& value) override;

    std::vector<sdbus::Struct<int32_t, int32_t>> ReactionWheelsData() override;

    void ReactionWheelsData(const std::vector<sdbus::Struct<int32_t, int32_t>>& value) override;

    sdbus::Struct<int32_t, int32_t> MagnetorquerData() override;

    void MagnetorquerData(const sdbus::Struct<int32_t, int32_t>& value) override;

private:
    sdbus::IObject& object_;
    StateMachine state_machine;
    std::mutex state_mutex;
};

#endif

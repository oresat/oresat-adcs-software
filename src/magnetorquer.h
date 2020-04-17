#include "adcs_data_structures.h"
#include "ADCS_dbus_adaptor.h"

class Magnetorquer {
    public :
        Magnetorquer(std::shared_ptr<ADCS_dbus_adaptor> input);
        ~Magnetorquer() = default;
        int detumble(const ADCS_data & adcs_data);
        int point(const ADCS_data & adcs_data);

    private :
        std::shared_ptr<ADCS_dbus_adaptor> dbus;
        ADCS_data last_date_frame;
};

#include "adcs_data_structures.h"
#include "ADCS_dbus_adaptor.h"

class ReactionWheels {
    public :
        ReactionWheels(std::shared_ptr<ADCS_dbus_adaptor> input);
        ~ReactionWheels() = default;
        int detumble(const ADCS_data & adcs_data_frame);
        int point(const ADCS_data & adcs_data_frame);

    private :
        std::shared_ptr<ADCS_dbus_adaptor> dbus;
        ADCS_data last_date_frame;
};

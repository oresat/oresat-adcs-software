#include <chrono>


struct GPS_Data {
    double pos_x;
    double pos_z;
    double pos_y;
    double vel_x;
    double vel_z;
    double vel_y;
    std::chrono::system_clock::time_point timestamp;
};


struct StarTrackerData {
    double right_ascension;
    double declination;
    double orientation;
    std::chrono::system_clock::time_point timestamp;
};


struct MagnometerData {
    int32_t foo;
    std::chrono::system_clock::time_point timestamp;
};


struct ReactionWheelData {
    int32_t bar;
    std::chrono::system_clock::time_point timestamp;
};


struct MagnetorquerData {
    int32_t baz;
    std::chrono::system_clock::time_point timestamp;
};


struct ADCS_Data {
    GPS_Data gps;
    StarTrackerData star_tracker;
    MagnometerData magnometer_pos_z;
    MagnometerData magnometer_neg_z;
    std::array<ReactionWheelData> reaction_wheels[4];
    MagnetorquerData magnetorquer;
};


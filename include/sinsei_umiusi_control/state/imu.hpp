#ifndef SINSEI_UMIUSI_CONTROL_STATE_IMU_HPP
#define SINSEI_UMIUSI_CONTROL_STATE_IMU_HPP

#include <cstdint>

namespace sinsei_umiusi_control::state::imu {

struct Quaternion {
    double x;
    double y;
    double z;
    double w;
};

struct Acceleration {
    double x;
    double y;
    double z;
};

struct AngularVelocity {
    double x;
    double y;
    double z;
};

struct Temperature {
    int8_t value;
};

struct Health {
    bool is_ok;
};

}  // namespace sinsei_umiusi_control::state::imu

#endif  // SINSEI_UMIUSI_CONTROL_STATE_IMU_HPP

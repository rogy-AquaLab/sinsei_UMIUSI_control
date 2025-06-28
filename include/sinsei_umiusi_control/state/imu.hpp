#include <cstdint>
#ifndef SINSEI_UMIUSI_CONTROL_STATE_IMU_HPP
#define SINSEI_UMIUSI_CONTROL_STATE_IMU_HPP

namespace sinsei_umiusi_control::state::imu {

struct Orientation {
    double x;
    double y;
    double z;
};
struct Velocity {
    double x;
    double y;
    double z;
};
struct Temperature {
    int8_t value;
};

struct ImuState {
    Orientation orientation;
    Velocity velocity;
    Temperature temperature;
};

}  // namespace sinsei_umiusi_control::state::imu

#endif  // SINSEI_UMIUSI_CONTROL_STATE_IMU_HPP
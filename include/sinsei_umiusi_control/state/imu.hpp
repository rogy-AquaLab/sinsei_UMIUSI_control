#include <cstdint>
#ifndef SINSEI_UMIUSI_CONTROL_STATE_IMU_HPP
#define SINSEI_UMIUSI_CONTROL_STATE_IMU_HPP

namespace sinsei_umiusi_control::state::imu {

struct Orientation {
    float x;
    float y;
    float z;
};
struct Velocity {
    float x;
    float y;
    float z;
};
struct Temperature {
    int8_t value;
};

}  // namespace sinsei_umiusi_control::state::imu

#endif  // SINSEI_UMIUSI_CONTROL_STATE_IMU_HPP
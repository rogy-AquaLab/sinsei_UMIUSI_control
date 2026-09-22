#ifndef SINSEI_UMIUSI_CONTROL_STATE_BMS_HPP
#define SINSEI_UMIUSI_CONTROL_STATE_BMS_HPP

#include <array>
#include <cstdint>

namespace sinsei_umiusi_control::state::bms {

struct Scalar {
    double value;
};

struct Boolean {
    bool value;
};

struct Count {
    uint8_t value;
};

struct PowerSwitchState {
    uint8_t value;
};

struct FaultFlags {
    uint32_t value;
};

struct StatusChunk {
    std::array<char, 8> value;
};

}  // namespace sinsei_umiusi_control::state::bms

#endif  // SINSEI_UMIUSI_CONTROL_STATE_BMS_HPP

#ifndef SINSEI_UMIUSI_CONTROL_STATE_MAIN_POWER_HPP
#define SINSEI_UMIUSI_CONTROL_STATE_MAIN_POWER_HPP

#include <cstdint>

namespace sinsei_umiusi_control::state::main_power {

struct BatteryCurrent {
    double value;
};
struct BatteryVoltage {
    double value;
};
struct Temperature {
    int8_t value;
};
struct WaterLeaked {
    bool value;
};

}  // namespace sinsei_umiusi_control::state::main_power

#endif  // SINSEI_UMIUSI_CONTROL_STATE_MAIN_POWER_HPP
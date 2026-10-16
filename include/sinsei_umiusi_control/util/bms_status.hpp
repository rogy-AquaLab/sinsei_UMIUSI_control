#ifndef SINSEI_UMIUSI_CONTROL_UTIL_BMS_STATUS_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_BMS_STATUS_HPP

#include <cstdint>

namespace sinsei_umiusi_control::util {

enum class BmsPowerSwitchState : uint8_t {
    Unknown = 0,
    Initializing = 1,
    Off = 2,
    Precharge = 3,
    On = 4,
    Fault = 5,
};

struct BmsFaults {
    bool precharge;
    bool short_circuit;
    bool switch_over_temperature;
    bool charge_overcurrent;
};

inline auto has_bms_fault(const BmsFaults & faults) -> bool {
    return faults.precharge || faults.short_circuit || faults.switch_over_temperature ||
           faults.charge_overcurrent;
}

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_BMS_STATUS_HPP

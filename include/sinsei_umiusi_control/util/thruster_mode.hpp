#ifndef SINSEI_UMIUSI_CONTROL_UTIL_THRUSTER_MDOE_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_THRUSTER_MDOE_HPP

#include <cstdint>

namespace sinsei_umiusi_control::util {

enum class ThrusterMode : int8_t {
    Disabled = -1,
    Standby = 0,
    Runnable = 1,
};

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_THRUSTER_MODE_HPP

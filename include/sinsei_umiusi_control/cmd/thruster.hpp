#include <cstdint>
#ifndef SINSEI_UMIUSI_CONTROL_CMD_THRUSTER_HPP
#define SINSEI_UMIUSI_CONTROL_CMD_THRUSTER_HPP

namespace sinsei_umiusi_control::cmd::thruster {

struct ServoEnabled {
    bool value;
};
struct EscEnabled {
    bool value;
};
struct Angle {
    int8_t value;
};
struct Thrust {
    int8_t value;
};

}  // namespace sinsei_umiusi_control::cmd::thruster

#endif  // SINSEI_UMIUSI_CONTROL_CMD_THRUSTER_HPP
#include <cstdint>
#ifndef SINSEI_UMIUSI_CONTROL_CMD_THRUSTER_HPP
#define SINSEI_UMIUSI_CONTROL_CMD_THRUSTER_HPP

namespace sinsei_umiusi_control::cmd::thruster {

struct Enabled {
    double value;
};
struct Angle {
    double value;
};
struct Thrust {
    double value;
};

}  // namespace sinsei_umiusi_control::cmd::thruster

#endif  // SINSEI_UMIUSI_CONTROL_CMD_THRUSTER_HPP
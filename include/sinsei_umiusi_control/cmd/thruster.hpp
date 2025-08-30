#ifndef SINSEI_UMIUSI_CONTROL_CMD_THRUSTER_HPP
#define SINSEI_UMIUSI_CONTROL_CMD_THRUSTER_HPP

namespace sinsei_umiusi_control::cmd::thruster {

struct EscEnabled {
    bool value;
};
struct ServoEnabled {
    bool value;
};
struct DutyCycle {
    double value;
};
struct Angle {
    double value;
};

}  // namespace sinsei_umiusi_control::cmd::thruster

#endif  // SINSEI_UMIUSI_CONTROL_CMD_THRUSTER_HPP

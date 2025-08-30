#ifndef SINSEI_UMIUSI_CONTROL_STATE_THRUSTER_HPP
#define SINSEI_UMIUSI_CONTROL_STATE_THRUSTER_HPP

namespace sinsei_umiusi_control::state::thruster {

struct Rpm {
    double value;
};
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

}  // namespace sinsei_umiusi_control::state::thruster

#endif  // SINSEI_UMIUSI_CONTROL_STATE_THRUSTER_HPP

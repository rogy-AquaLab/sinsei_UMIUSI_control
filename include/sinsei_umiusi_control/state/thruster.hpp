#ifndef SINSEI_UMIUSI_CONTROL_STATE_THRUSTER_HPP
#define SINSEI_UMIUSI_CONTROL_STATE_THRUSTER_HPP

namespace sinsei_umiusi_control::state::thruster {

struct ServoCurrent {
    float value;
};
struct Rpm {
    float value;
};

}  // namespace sinsei_umiusi_control::state::thruster

#endif  // SINSEI_UMIUSI_CONTROL_STATE_THRUSTER_HPP
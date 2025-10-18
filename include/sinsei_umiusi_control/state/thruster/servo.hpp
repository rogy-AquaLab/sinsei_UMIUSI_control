#ifndef SINSEI_UMIUSI_CONTROL_STATE_THRUSTER_SERVO_HPP
#define SINSEI_UMIUSI_CONTROL_STATE_THRUSTER_SERVO_HPP

namespace sinsei_umiusi_control::state::thruster::servo {

struct Enabled {
    bool value;
};
struct Angle {
    double value;
};
struct Health {
    bool is_ok;
};

}  // namespace sinsei_umiusi_control::state::thruster::servo

#endif  // SINSEI_UMIUSI_CONTROL_STATE_THRUSTER_SERVO_HPP

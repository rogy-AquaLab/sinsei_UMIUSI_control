#ifndef SINSEI_UMIUSI_CONTROL_STATE_ACTUATOR_MOTOR_HPP
#define SINSEI_UMIUSI_CONTROL_STATE_ACTUATOR_MOTOR_HPP

namespace sinsei_umiusi_control::state::actuator::motor {

struct Rpm {
    double value;
};
struct Voltage {
    double value;
};
struct WaterLeaked {
    bool value;
};

}  // namespace sinsei_umiusi_control::state::actuator::motor

#endif  // SINSEI_UMIUSI_CONTROL_STATE_ACTUATOR_MOTOR_HPP

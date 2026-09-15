#ifndef SINSEI_UMIUSI_CONTROL_CMD_ACTUATOR_MOTOR_HPP
#define SINSEI_UMIUSI_CONTROL_CMD_ACTUATOR_MOTOR_HPP

namespace sinsei_umiusi_control::cmd::actuator::motor {

struct DutyCycle {
    double value;
};

// 最終的にモーターを動かしていいかどうか
struct Allowed {
    bool value;
};

}  // namespace sinsei_umiusi_control::cmd::actuator::motor

#endif  // SINSEI_UMIUSI_CONTROL_CMD_ACTUATOR_MOTOR_HPP

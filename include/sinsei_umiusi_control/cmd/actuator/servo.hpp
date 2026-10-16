#ifndef SINSEI_UMIUSI_CONTROL_CMD_ACTUATOR_SERVO_HPP
#define SINSEI_UMIUSI_CONTROL_CMD_ACTUATOR_SERVO_HPP

namespace sinsei_umiusi_control::cmd::actuator::servo {

struct Angle {
    double value;
};

// 最終的にサーボを動かしていいかどうか
struct Allowed {
    bool value;
};

}  // namespace sinsei_umiusi_control::cmd::actuator::servo

#endif  // SINSEI_UMIUSI_CONTROL_CMD_ACTUATOR_SERVO_HPP

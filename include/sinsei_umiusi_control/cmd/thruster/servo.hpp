#ifndef SINSEI_UMIUSI_CONTROL_CMD_THRUSTER_SERVO_HPP
#define SINSEI_UMIUSI_CONTROL_CMD_THRUSTER_SERVO_HPP

namespace sinsei_umiusi_control::cmd::thruster::servo {

struct Runnable {
    bool value;
};
struct Angle {
    double value;
};

// 最終的にスラスタを動かしていいかどうか
struct Allowed {
    bool value;
};

}  // namespace sinsei_umiusi_control::cmd::thruster::servo

#endif  // SINSEI_UMIUSI_CONTROL_CMD_THRUSTER_SERVO_HPP

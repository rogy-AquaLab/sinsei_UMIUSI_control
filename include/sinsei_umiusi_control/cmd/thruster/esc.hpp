#ifndef SINSEI_UMIUSI_CONTROL_CMD_THRUSTER_ESC_HPP
#define SINSEI_UMIUSI_CONTROL_CMD_THRUSTER_ESC_HPP

namespace sinsei_umiusi_control::cmd::thruster::esc {

struct Runnable {
    bool value;
};
struct DutyCycle {
    double value;
};
struct Thrust {  // 推力
    double value;
};

// 最終的にスラスタを動かしていいかどうか
struct Allowed {
    bool value;
};

}  // namespace sinsei_umiusi_control::cmd::thruster::esc

#endif  // SINSEI_UMIUSI_CONTROL_CMD_THRUSTER_ESC_HPP

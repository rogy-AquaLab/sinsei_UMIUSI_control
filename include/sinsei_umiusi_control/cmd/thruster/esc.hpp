#ifndef SINSEI_UMIUSI_CONTROL_CMD_THRUSTER_ESC_HPP
#define SINSEI_UMIUSI_CONTROL_CMD_THRUSTER_ESC_HPP

#include "sinsei_umiusi_control/cmd/actuator/motor.hpp"

namespace sinsei_umiusi_control::cmd::thruster::esc {

struct Runnable {
    bool value;
};
using DutyCycle = cmd::actuator::motor::DutyCycle;
struct Thrust {  // 推力
    double value;
};

using Allowed = cmd::actuator::motor::Allowed;

}  // namespace sinsei_umiusi_control::cmd::thruster::esc

#endif  // SINSEI_UMIUSI_CONTROL_CMD_THRUSTER_ESC_HPP

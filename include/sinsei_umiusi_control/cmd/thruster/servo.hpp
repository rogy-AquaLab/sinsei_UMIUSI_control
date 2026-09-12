#ifndef SINSEI_UMIUSI_CONTROL_CMD_THRUSTER_SERVO_HPP
#define SINSEI_UMIUSI_CONTROL_CMD_THRUSTER_SERVO_HPP

#include "sinsei_umiusi_control/cmd/actuator/servo.hpp"

namespace sinsei_umiusi_control::cmd::thruster::servo {

struct Runnable {
    bool value;
};
using Angle = cmd::actuator::servo::Angle;
using Allowed = cmd::actuator::servo::Allowed;

}  // namespace sinsei_umiusi_control::cmd::thruster::servo

#endif  // SINSEI_UMIUSI_CONTROL_CMD_THRUSTER_SERVO_HPP

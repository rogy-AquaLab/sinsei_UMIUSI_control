#ifndef SINSEI_UMIUSI_CONTROL_STATE_THRUSTER_ESC_HPP
#define SINSEI_UMIUSI_CONTROL_STATE_THRUSTER_ESC_HPP

#include "sinsei_umiusi_control/state/actuator/motor.hpp"
#include "sinsei_umiusi_control/util/thruster_mode.hpp"

namespace sinsei_umiusi_control::state::thruster::esc {

struct Mode {
    util::ThrusterMode value;
};
struct DutyCycle {
    double value;
};
using Rpm = state::actuator::motor::Rpm;
using Voltage = state::actuator::motor::Voltage;
using WaterLeaked = state::actuator::motor::WaterLeaked;

}  // namespace sinsei_umiusi_control::state::thruster::esc

#endif  // SINSEI_UMIUSI_CONTROL_STATE_THRUSTER_ESC_HPP

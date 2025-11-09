#ifndef SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_THRUSTER_LINEAR_ACCELERATION_HPP
#define SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_THRUSTER_LINEAR_ACCELERATION_HPP

#include <algorithm>

#include "sinsei_umiusi_control/controller/thruster_controller.hpp"

namespace sinsei_umiusi_control::controller::logic::thruster {

class LinearAcceleration : public ThrusterController::Logic {
  private:
    double duty_per_thrust;
    double max_duty_cycle;
    double max_duty_step_per_sec;
    double duty_cycle;

  public:
    LinearAcceleration(
        double duty_per_thrust, double max_duty_cycle, double max_duty_step_per_sec)  // NOLINT
    : duty_per_thrust(duty_per_thrust),
      max_duty_cycle(max_duty_cycle),
      max_duty_step_per_sec(max_duty_step_per_sec),
      duty_cycle(0.0) {}

    auto control_mode() const -> logic::ControlMode override {
        return logic::ControlMode::FeedForward;
    }

    auto init(
        double /*time*/, const ThrusterController::Input & /*input*/,
        const ThrusterController::Output & output) -> ThrusterController::Output override {
        return output;
    }

    auto update(double /*time*/, double duration, const ThrusterController::Input & input)
        -> ThrusterController::Output override {
        // Calculate the target duty cycle within limits
        const auto step_limit = this->max_duty_step_per_sec * duration;
        const auto min = std::max(-this->max_duty_cycle, this->duty_cycle - step_limit);
        const auto max = std::min(this->max_duty_cycle, this->duty_cycle + step_limit);
        const auto target = this->duty_per_thrust * input.cmd.esc_thrust.value;
        this->duty_cycle = std::clamp(target, min, max);

        auto output = ThrusterController::Output{};
        output.state.esc_enabled.value = input.cmd.esc_enabled.value;
        output.state.esc_duty_cycle.value = this->duty_cycle;
        output.state.servo_enabled.value = input.cmd.servo_enabled.value;
        output.state.servo_angle.value = input.cmd.servo_angle.value;
        return output;
    }
};

}  // namespace sinsei_umiusi_control::controller::logic::thruster

#endif  // SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_THRUSTER_LINEAR_ACCELERATION_HPP

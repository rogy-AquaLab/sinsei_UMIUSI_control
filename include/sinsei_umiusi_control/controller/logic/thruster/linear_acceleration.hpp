#ifndef SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_THRUSTER_LINEAR_ACCELERATION_HPP
#define SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_THRUSTER_LINEAR_ACCELERATION_HPP

#include <algorithm>

#include "sinsei_umiusi_control/controller/thruster_controller.hpp"

namespace sinsei_umiusi_control::controller::logic::thruster {

class LinearAcceleration : public ThrusterController::Logic {
  public:
    static constexpr auto DEFAULT_MAX_DUTY_CYCLE = 0.5;
    static constexpr auto DEFAULT_MAX_DUTY_STEP_PER_SEC = 1.0;
    static constexpr auto DEFAULT_DUTY_PER_THRUST = 1.0;

    double max_duty_cycle;
    double max_duty_step_per_sec;
    double duty_per_thrust;
    double duty_cycle;

    LinearAcceleration()
    : max_duty_cycle(DEFAULT_MAX_DUTY_CYCLE),
      max_duty_step_per_sec(DEFAULT_MAX_DUTY_STEP_PER_SEC),
      duty_per_thrust(DEFAULT_DUTY_PER_THRUST),
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

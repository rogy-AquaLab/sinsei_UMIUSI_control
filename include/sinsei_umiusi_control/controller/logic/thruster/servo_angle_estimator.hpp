#ifndef SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_THRUSTER_SERVO_ANGLE_ESTIMATOR_HPP
#define SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_THRUSTER_SERVO_ANGLE_ESTIMATOR_HPP

#include <optional>

#include "sinsei_umiusi_control/state/thruster/servo.hpp"

namespace sinsei_umiusi_control::controller::logic::thruster {

class ServoAngleEstimator {
  private:
    double max_angular_velocity;
    double min_possible_angle;
    double max_possible_angle;

    static auto move_towards(double current, double target, double max_delta) -> double;

  public:
    explicit ServoAngleEstimator(double max_angular_velocity);

    auto update(state::thruster::servo::CommandedAngle commanded_angle, double duration)
        -> std::optional<state::thruster::servo::EstimatedAngle>;
    void reset();
};

}  // namespace sinsei_umiusi_control::controller::logic::thruster

#endif  // SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_THRUSTER_SERVO_ANGLE_ESTIMATOR_HPP

#include "sinsei_umiusi_control/controller/logic/thruster/servo_angle_estimator.hpp"

#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <stdexcept>

using namespace sinsei_umiusi_control::controller::logic::thruster;

namespace {
constexpr auto HALF_PI = boost::math::constants::pi<double>() / 2.0;
constexpr auto MIN_ANGLE = -HALF_PI;
constexpr auto MAX_ANGLE = HALF_PI;
}  // namespace

ServoAngleEstimator::ServoAngleEstimator(double max_angular_velocity)
: max_angular_velocity(max_angular_velocity) {
    if (!std::isfinite(max_angular_velocity) || max_angular_velocity < 0.0) {
        throw std::invalid_argument("max_angular_velocity must be finite and non-negative");
    }
    this->reset();
}

auto ServoAngleEstimator::move_towards(double current, double target, double max_delta) -> double {
    if (current < target) {
        return std::min(current + max_delta, target);
    }
    if (current > target) {
        return std::max(current - max_delta, target);
    }
    return target;
}

auto ServoAngleEstimator::update(
    state::thruster::servo::CommandedAngle commanded_angle,
    double duration) -> std::optional<state::thruster::servo::EstimatedAngle> {
    if (!std::isfinite(commanded_angle.value) || commanded_angle.value < MIN_ANGLE ||
        commanded_angle.value > MAX_ANGLE || !std::isfinite(duration) || duration < 0.0) {
        this->reset();
        return std::nullopt;
    }

    const auto max_delta = std::min(this->max_angular_velocity * duration, MAX_ANGLE - MIN_ANGLE);
    this->min_possible_angle =
        move_towards(this->min_possible_angle, commanded_angle.value, max_delta);
    this->max_possible_angle =
        move_towards(this->max_possible_angle, commanded_angle.value, max_delta);

    if (this->min_possible_angle == this->max_possible_angle) {
        return state::thruster::servo::EstimatedAngle{this->min_possible_angle};
    }
    return std::nullopt;
}

void ServoAngleEstimator::reset() {
    this->min_possible_angle = MIN_ANGLE;
    this->max_possible_angle = MAX_ANGLE;
}

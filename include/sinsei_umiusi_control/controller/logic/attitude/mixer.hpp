#ifndef SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_MIXER_HPP
#define SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_MIXER_HPP

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <optional>

#include "sinsei_umiusi_control/controller/attitude_controller.hpp"

namespace sinsei_umiusi_control::controller::logic::attitude {

namespace detail {

constexpr auto PI = boost::math::constants::pi<double>();
constexpr auto HALF_PI = PI / 2.0;
constexpr auto SERVO_DIRECTION_DEADBAND = PI / 90.0;  // 2 deg
constexpr auto SERVO_REVERSAL_DEADBAND = PI / 18.0;   // 10 deg

inline auto canonical_servo_angle(double horizontal, double vertical) -> double {
    auto angle = std::atan2(vertical, horizontal);
    if (angle > HALF_PI) {
        angle -= PI;
    } else if (angle < -HALF_PI) {
        angle += PI;
    }
    return angle;
}

inline auto target_servo_angle(double horizontal, double vertical, double current_angle) -> double {
    if (horizontal == 0.0 && vertical == 0.0) {
        return current_angle;
    }

    const auto exact_angle = canonical_servo_angle(horizontal, vertical);
    const auto direct_distance = std::abs(exact_angle - current_angle);
    const auto axis_distance = std::min(direct_distance, PI - direct_distance);

    // 小さな方向変化は現在角での推力射影に任せ、サーボの微動を抑える。
    if (axis_distance <= SERVO_DIRECTION_DEADBAND) {
        return current_angle;
    }

    // ±90 deg は、ESC の符号を反転すればほぼ同じ推力軸を表せる。
    // 境界のごく近傍では反対側へ180 deg回さず、現在側の端点を維持する。
    if (current_angle * exact_angle < 0.0 && direct_distance > HALF_PI &&
        PI - direct_distance <= SERVO_REVERSAL_DEADBAND) {
        return std::copysign(HALF_PI, current_angle);
    }
    return exact_angle;
}

inline auto move_towards(double current, double target, double max_delta) -> double {
    return std::clamp(target, current - max_delta, current + max_delta);
}

}  // namespace detail

inline auto hold_current_servo_angles(
    const std::array<std::optional<state::thruster::servo::EstimatedAngle>, 4> &
        servo_estimated_angles) -> AttitudeController::Output {
    auto output = AttitudeController::Output{};
    for (size_t i = 0; i < servo_estimated_angles.size(); ++i) {
        if (!servo_estimated_angles[i] || !std::isfinite(servo_estimated_angles[i]->value) ||
            servo_estimated_angles[i]->value < -detail::HALF_PI ||
            servo_estimated_angles[i]->value > detail::HALF_PI) {
            return {};
        }
        output.cmd.servo_angles[i].value = servo_estimated_angles[i]->value;
    }
    return output;
}

// u = [θx, θy, θz, vx, vy, vz] (x/y/z軸まわりのモーメント要求、x/y/z方向の並進力要求)
inline auto mix_to_thrusters(
    const Eigen::Vector<double, 6> & u,
    const std::array<std::optional<state::thruster::servo::EstimatedAngle>, 4> &
        servo_estimated_angles,
    const std::array<state::thruster::servo::MaxAngularVelocity, 4> & servo_max_angular_velocities,
    double duration) -> AttitudeController::Output {
    auto output = hold_current_servo_angles(servo_estimated_angles);
    if (!u.allFinite() || !std::isfinite(duration) || duration < 0.0) {
        return output;
    }

    // 現在角が確定するまでは全基を0 degへ初期化し、推力を出さない。
    // 1基だけ先に推力を出すと意図しない合力・モーメントになるため、全基が揃うまで待つ。
    for (size_t i = 0; i < servo_estimated_angles.size(); ++i) {
        if (!servo_estimated_angles[i] || !std::isfinite(servo_estimated_angles[i]->value) ||
            servo_estimated_angles[i]->value < -detail::HALF_PI ||
            servo_estimated_angles[i]->value > detail::HALF_PI ||
            !std::isfinite(servo_max_angular_velocities[i].value) ||
            servo_max_angular_velocities[i].value < 0.0) {
            return {};
        }
    }

    //  -> f1h, f1v, f2h, f2v, f3h, f3v, f4h, f4v (h: horizontal, v: vertical)
    // z軸まわりに半時計周りをfの番号順とhorizontalの正の方向とする
    const auto a = Eigen::Matrix<double, 8, 6>{
        {0.0, 0.0, 1.0, -sqrt(2.0), sqrt(2.0), 0.0},   // スラスタ1 (:lf) 水平出力
        {1.0, -1.0, 0.0, 0.0, 0.0, 1.0},               // スラスタ1 (:lf) 垂直出力
        {0.0, 0.0, 1.0, -sqrt(2.0), -sqrt(2.0), 0.0},  // スラスタ2 (:lb) 水平出力
        {1.0, 1.0, 0.0, 0.0, 0.0, 1.0},                // スラスタ2 (:lb) 垂直出力
        {0.0, 0.0, 1.0, sqrt(2.0), -sqrt(2.0), 0.0},   // スラスタ3 (:rb) 水平出力
        {-1.0, 1.0, 0.0, 0.0, 0.0, 1.0},               // スラスタ3 (:rb) 垂直出力
        {0.0, 0.0, 1.0, sqrt(2.0), sqrt(2.0), 0.0},    // スラスタ4 (:rf) 水平出力
        {-1.0, -1.0, 0.0, 0.0, 0.0, 1.0},              // スラスタ4 (:rf) 垂直出力
    };
    const auto y = a * u;

    constexpr auto MAX_THRUST = boost::math::constants::root_two<double>();
    for (size_t i = 0; i < servo_estimated_angles.size(); ++i) {
        const auto horizontal = y[2 * i];
        const auto vertical = y[2 * i + 1];
        const auto current_angle = servo_estimated_angles[i]->value;
        const auto target_angle = detail::target_servo_angle(horizontal, vertical, current_angle);
        const auto max_angle_step =
            std::min(servo_max_angular_velocities[i].value * duration, detail::PI);
        const auto commanded_angle = std::clamp(
            detail::move_towards(current_angle, target_angle, max_angle_step), -detail::HALF_PI,
            detail::HALF_PI);

        // サーボがまだ指令角へ着いていない間も、現在の推力軸へ要求ベクトルを射影する。
        // これにより「サーボだけ追従中、ESCは到達後の角度を前提に出力」という不整合を防ぐ。
        const auto projected_thrust =
            horizontal * std::cos(current_angle) + vertical * std::sin(current_angle);
        output.cmd.servo_angles[i].value = commanded_angle;
        output.cmd.esc_thrusts[i].value = projected_thrust / MAX_THRUST;
    }

    return output;
}

}  // namespace sinsei_umiusi_control::controller::logic::attitude

#endif  // SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_MIXER_HPP

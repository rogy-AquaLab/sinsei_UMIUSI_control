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

}  // namespace detail

struct MixerParameters {
    double servo_direction_deadband;
    double servo_reversal_deadband;
    double servo_retarget_thrust_enter;
    double servo_retarget_thrust_exit;
    // ESC 推力（正規化）の上限。thruster 側の max_duty / duty_per_thrust に合わせること。
    double esc_thrust_limit = 1.0;
};

struct MixerState {
    std::array<bool, 4> servo_retargeting{};
};

namespace detail {

inline auto canonical_servo_angle(double horizontal, double vertical) -> double {
    auto angle = std::atan2(vertical, horizontal);
    if (angle > HALF_PI) {
        angle -= PI;
    } else if (angle < -HALF_PI) {
        angle += PI;
    }
    return angle;
}

inline auto target_servo_angle(
    double horizontal, double vertical, double current_angle,
    const MixerParameters & parameters) -> double {
    if (horizontal == 0.0 && vertical == 0.0) {
        return current_angle;
    }

    const auto exact_angle = canonical_servo_angle(horizontal, vertical);
    const auto direct_distance = std::abs(exact_angle - current_angle);
    const auto axis_distance = std::min(direct_distance, PI - direct_distance);

    // 小さな方向変化は現在角での推力射影に任せ、サーボの微動を抑える。
    if (axis_distance <= parameters.servo_direction_deadband) {
        return current_angle;
    }

    // ±90 deg は、ESC の符号を反転すればほぼ同じ推力軸を表せる。
    // 目標が境界のごく近傍なら反対側へ回さず、現在角と同じ側の端点を目標にする。
    // 判定を現在角ではなく目標角の境界距離で行うのは、垂直推力に yaw 等の小さな水平成分が
    // 正負に揺れて乗ると目標が ±(90 - ε) deg で反転し続け、端点から遠い現在角
    // (0 deg 付近) のサーボが往復するだけで垂直へ向かえなくなるため。
    if (current_angle * exact_angle < 0.0 &&
        HALF_PI - std::abs(exact_angle) <= parameters.servo_reversal_deadband) {
        return std::copysign(HALF_PI, current_angle);
    }
    return exact_angle;
}

inline auto move_towards(double current, double target, double max_delta) -> double {
    return std::clamp(target, current - max_delta, current + max_delta);
}

// |moment + s * translation| <= limit を満たす最大の s (0..1)。|moment| <= limit が前提。
inline auto translation_scale(
    const Eigen::Vector2d & moment, const Eigen::Vector2d & translation, double limit) -> double {
    const auto tt = translation.squaredNorm();
    if (tt == 0.0) {
        return 1.0;
    }
    const auto mt = moment.dot(translation);
    const auto discriminant = mt * mt - tt * (moment.squaredNorm() - limit * limit);
    return std::clamp((-mt + std::sqrt(std::max(discriminant, 0.0))) / tt, 0.0, 1.0);
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
    double duration, const MixerParameters & parameters,
    MixerState & mixer_state) -> AttitudeController::Output {
    auto output = hold_current_servo_angles(servo_estimated_angles);
    if (!u.allFinite() || !std::isfinite(duration) || duration < 0.0) {
        mixer_state.servo_retargeting.fill(false);
        return output;
    }
    if (!std::isfinite(parameters.servo_direction_deadband) ||
        parameters.servo_direction_deadband < 0.0 ||
        parameters.servo_direction_deadband > detail::HALF_PI ||
        !std::isfinite(parameters.servo_reversal_deadband) ||
        parameters.servo_reversal_deadband < 0.0 ||
        parameters.servo_reversal_deadband > detail::HALF_PI ||
        !std::isfinite(parameters.servo_retarget_thrust_enter) ||
        parameters.servo_retarget_thrust_enter < 0.0 ||
        parameters.servo_retarget_thrust_enter > 1.0 ||
        !std::isfinite(parameters.servo_retarget_thrust_exit) ||
        parameters.servo_retarget_thrust_exit < 0.0 ||
        parameters.servo_retarget_thrust_exit > parameters.servo_retarget_thrust_enter ||
        !std::isfinite(parameters.esc_thrust_limit) || parameters.esc_thrust_limit <= 0.0 ||
        parameters.esc_thrust_limit > 1.0) {
        mixer_state.servo_retargeting.fill(false);
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
            mixer_state.servo_retargeting.fill(false);
            return {};
        }
    }

    //  -> f1h, f1v, f2h, f2v, f3h, f3v, f4h, f4v (h: horizontal, v: vertical)
    // z軸まわりに半時計周りをfの番号順とhorizontalの正の方向とする
    // 並進 (vx, vy, vz) は軸によらず「入力 1.0 で各基の正規化推力 1.0」になるよう、
    // 列を MAX_THRUST (= √2) 倍にそろえる。z だけ 1 倍だと同じ入力で xy の 1/√2 しか出ない。
    const auto a = Eigen::Matrix<double, 8, 6>{
        {0.0, 0.0, 1.0, -sqrt(2.0), sqrt(2.0), 0.0},   // スラスタ1 (:lf) 水平出力
        {1.0, -1.0, 0.0, 0.0, 0.0, sqrt(2.0)},         // スラスタ1 (:lf) 垂直出力
        {0.0, 0.0, 1.0, -sqrt(2.0), -sqrt(2.0), 0.0},  // スラスタ2 (:lb) 水平出力
        {1.0, 1.0, 0.0, 0.0, 0.0, sqrt(2.0)},          // スラスタ2 (:lb) 垂直出力
        {0.0, 0.0, 1.0, sqrt(2.0), -sqrt(2.0), 0.0},   // スラスタ3 (:rb) 水平出力
        {-1.0, 1.0, 0.0, 0.0, 0.0, sqrt(2.0)},         // スラスタ3 (:rb) 垂直出力
        {0.0, 0.0, 1.0, sqrt(2.0), sqrt(2.0), 0.0},    // スラスタ4 (:rf) 水平出力
        {-1.0, -1.0, 0.0, 0.0, 0.0, sqrt(2.0)},        // スラスタ4 (:rf) 垂直出力
    };

    // 上限超えを thruster 側の clip に任せると全基が同じ値に張り付き、姿勢モーメントが消える。
    constexpr auto MAX_THRUST = boost::math::constants::root_two<double>();
    const auto limit = parameters.esc_thrust_limit * MAX_THRUST;
    auto u_moment = u;
    u_moment.tail<3>().setZero();
    auto u_translation = u;
    u_translation.head<3>().setZero();
    Eigen::Vector<double, 8> y_moment = a * u_moment;
    const Eigen::Vector<double, 8> y_translation = a * u_translation;
    auto moment_peak = 0.0;
    for (size_t i = 0; i < servo_estimated_angles.size(); ++i) {
        moment_peak = std::max(moment_peak, y_moment.segment<2>(2 * i).norm());
    }
    if (moment_peak > limit) {
        y_moment *= limit / moment_peak;
    }
    auto scale = 1.0;
    for (size_t i = 0; i < servo_estimated_angles.size(); ++i) {
        scale = std::min(
            scale, detail::translation_scale(
                       y_moment.segment<2>(2 * i), y_translation.segment<2>(2 * i), limit));
    }
    const Eigen::Vector<double, 8> y = y_moment + scale * y_translation;

    for (size_t i = 0; i < servo_estimated_angles.size(); ++i) {
        const auto horizontal = y[2 * i];
        const auto vertical = y[2 * i + 1];
        const auto current_angle = servo_estimated_angles[i]->value;
        // 小推力域では現在角を維持する。開始・停止の閾値を分けることで、
        // 閾値付近のノイズによる追従状態の頻繁な切り替えを防ぐ。
        const auto requested_thrust = std::hypot(horizontal, vertical) / MAX_THRUST;
        if (mixer_state.servo_retargeting[i]) {
            if (requested_thrust <= parameters.servo_retarget_thrust_exit) {
                mixer_state.servo_retargeting[i] = false;
            }
        } else if (requested_thrust >= parameters.servo_retarget_thrust_enter) {
            mixer_state.servo_retargeting[i] = true;
        }
        const auto target_angle = mixer_state.servo_retargeting[i]
                                      ? detail::target_servo_angle(
                                            horizontal, vertical, current_angle, parameters)
                                      : current_angle;
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

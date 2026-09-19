#ifndef SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_MIXER_HPP
#define SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_MIXER_HPP

#include <Eigen/Core>
#include <array>
#include <boost/math/constants/constants.hpp>
#include <cmath>

#include "sinsei_umiusi_control/controller/attitude_controller.hpp"

namespace sinsei_umiusi_control::controller::logic::attitude {

namespace detail {

inline auto atan_or_zero(const double & x, const double & y) -> double {
    if (x == 0.0 && y == 0.0) {
        return 0.0;
    }
    return std::atan(y / x);
}

inline auto magnitude(const double & x, const double & y) -> double {
    return std::sqrt(x * x + y * y);
}

inline auto sign(const double & x) -> double { return x < 0.0 ? -1.0 : 1.0; }

}  // namespace detail

// u = [θx, θy, θz, vx, vy, vz] (x/y/z軸まわりのモーメント要求、x/y/z方向の並進力要求)
inline auto mix_to_thrusters(const Eigen::Vector<double, 6> & u) -> AttitudeController::Output {
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

    auto output = AttitudeController::Output{};
    output.cmd.servo_angles = {
        sinsei_umiusi_control::cmd::thruster::servo::Angle{detail::atan_or_zero(y[0], y[1])},
        sinsei_umiusi_control::cmd::thruster::servo::Angle{detail::atan_or_zero(y[2], y[3])},
        sinsei_umiusi_control::cmd::thruster::servo::Angle{detail::atan_or_zero(y[4], y[5])},
        sinsei_umiusi_control::cmd::thruster::servo::Angle{detail::atan_or_zero(y[6], y[7])},
    };
    const auto thrust_sgns = std::array<double, 4>{
        detail::sign(y[0]), detail::sign(y[2]), detail::sign(y[4]), detail::sign(y[6])};
    const auto thrust_abss = std::array<double, 4>{
        detail::magnitude(y[0], y[1]), detail::magnitude(y[2], y[3]), detail::magnitude(y[4], y[5]),
        detail::magnitude(y[6], y[7])};

    // 絶対値の最大値が1になるように正規化
    // √2 == (a * 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 の第一成分と第二成分の二乗和の平方根)
    constexpr auto MAX_THRUST = boost::math::constants::root_two<double>();
    output.cmd.esc_thrusts = {
        sinsei_umiusi_control::cmd::thruster::esc::Thrust{
            thrust_sgns[0] * thrust_abss[0] / MAX_THRUST},
        sinsei_umiusi_control::cmd::thruster::esc::Thrust{
            thrust_sgns[1] * thrust_abss[1] / MAX_THRUST},
        sinsei_umiusi_control::cmd::thruster::esc::Thrust{
            thrust_sgns[2] * thrust_abss[2] / MAX_THRUST},
        sinsei_umiusi_control::cmd::thruster::esc::Thrust{
            thrust_sgns[3] * thrust_abss[3] / MAX_THRUST},
    };

    return output;
}

}  // namespace sinsei_umiusi_control::controller::logic::attitude

#endif  // SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_MIXER_HPP

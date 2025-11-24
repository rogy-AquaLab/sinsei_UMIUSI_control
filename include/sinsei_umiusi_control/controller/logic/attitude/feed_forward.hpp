#ifndef SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_FEED_FORWARD_HPP
#define SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_FEED_FORWARD_HPP

#include <Eigen/Core>
#include <boost/math/constants/constants.hpp>

#include "sinsei_umiusi_control/controller/attitude_controller.hpp"
#include "sinsei_umiusi_control/controller/logic/attitude/data_conversion.hpp"

namespace sinsei_umiusi_control::controller::logic::attitude {

class FeedForward : public AttitudeController::Logic {
  private:
    static auto atan_or_zero(const double & x, const double & y) -> double {
        constexpr auto pi = boost::math::constants::pi<double>();
        if (x == 0.0 && y == 0.0) {
            return 0.0;
        }
        return std::atan(y / x) * 180.0 / pi;
    }

    static auto magnitude(const double & x, const double & y) -> double {
        return std::sqrt(x * x + y * y);
    }

    static auto sign(const double & x, const double & y) -> double {
        // π/2 < φ < 3π/2 -> negative
        constexpr auto half_pi = boost::math::constants::half_pi<double>();
        const auto phi = std::atan2(y, x);
        return (phi > half_pi && phi < 3.0 * half_pi) ? -1.0 : 1.0;
    }

  public:
    auto control_mode() const -> logic::ControlMode override {
        return logic::ControlMode::FeedForward;
    }

    auto init(
        double /*time*/, const AttitudeController::Input & /*input*/,
        const AttitudeController::Output & output) -> AttitudeController::Output override {
        return output;
    }

    auto update(double /*time*/, double /*duration*/, const AttitudeController::Input & input)
        -> AttitudeController::Output override {
        const auto target_orientation = to_eigen_vector(input.cmd.target_orientation);
        const auto target_velocity = to_eigen_vector(input.cmd.target_velocity);

        // θx, θy, θz, vx, vy, vz
        //  -> f1h, f1v, f2h, f2v, f3h, f3v, f4h, f4v (h: horizontal, v: vertical)
        // z軸まわりに半時計周りをfの番号順とhorizontalの正の方向とする

        // 変換前の行列
        const auto u = Eigen::Vector<double, 6>{
            target_orientation[0],  // 目標姿勢ベクトル
            target_orientation[1],  //
            target_orientation[2],  //
            target_velocity[0],     // 目標速度ベクトル
            target_velocity[1],     //
            target_velocity[2],     //
        };
        // 係数行列
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
            sinsei_umiusi_control::cmd::thruster::servo::Angle{atan_or_zero(y[0], y[1])},
            sinsei_umiusi_control::cmd::thruster::servo::Angle{atan_or_zero(y[2], y[3])},
            sinsei_umiusi_control::cmd::thruster::servo::Angle{atan_or_zero(y[4], y[5])},
            sinsei_umiusi_control::cmd::thruster::servo::Angle{atan_or_zero(y[6], y[7])},
        };
        const auto thrust_sgns = std::array<double, 4>{
            sign(y[0], y[1]), sign(y[2], y[3]), sign(y[4], y[5]), sign(y[6], y[7])};
        const auto thrust_abss = std::array<double, 4>{
            magnitude(y[0], y[1]), magnitude(y[2], y[3]), magnitude(y[4], y[5]),
            magnitude(y[6], y[7])};

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
};

}  // namespace sinsei_umiusi_control::controller::logic::attitude

#endif  // SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_FEED_FORWARD_HPP

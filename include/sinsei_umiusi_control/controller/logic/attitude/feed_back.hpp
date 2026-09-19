#ifndef SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_FEED_BACK_HPP
#define SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_FEED_BACK_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sinsei_umiusi_control/controller/attitude_controller.hpp"
#include "sinsei_umiusi_control/controller/logic/attitude/data_conversion.hpp"
#include "sinsei_umiusi_control/controller/logic/attitude/mixer.hpp"

namespace sinsei_umiusi_control::controller::logic::attitude {

// IMU実測姿勢との誤差から、ロール・ピッチはP制御、ヨーはレートのP制御でモーメント要求を計算する
class FeedBack : public AttitudeController::Logic {
  private:
    // ロール・ピッチの姿勢誤差(クオータニオン誤差のベクトル部)からモーメント要求への変換ゲイン
    static constexpr double KP_ATTITUDE = 1.0;
    // ヨーレート誤差(目標レート - IMU角速度)からモーメント要求への変換ゲイン
    static constexpr double KP_YAW_RATE = 1.0;

  public:
    auto control_mode() const -> logic::ControlMode override {
        return logic::ControlMode::FeedBack;
    }

    auto init(
        double /*time*/, const AttitudeController::Input & /*input*/,
        const AttitudeController::Output & output) -> AttitudeController::Output override {
        return output;
    }

    auto update(double /*time*/, double /*duration*/, const AttitudeController::Input & input)
        -> AttitudeController::Output override {
        const auto target_attitude = to_eigen_quaternion(input.cmd.target_attitude);
        const auto current_attitude = to_eigen_quaternion(input.state.imu_quaternion);
        const auto target_velocity = to_eigen_vector(input.cmd.target_velocity);

        // 現在姿勢から見て目標姿勢に到達するために必要な回転(小角近似ではベクトル部がロール・ピッチ誤差角に比例)
        const auto attitude_error = (current_attitude.inverse() * target_attitude).vec();

        const auto yaw_rate_error =
            input.cmd.target_attitude.yaw_rate - input.state.imu_angular_velocity.z;

        const auto u = Eigen::Vector<double, 6>{
            KP_ATTITUDE * attitude_error.x(),  // ロール誤差
            KP_ATTITUDE * attitude_error.y(),  // ピッチ誤差
            KP_YAW_RATE * yaw_rate_error,       // ヨーレート誤差
            target_velocity[0],                 // 目標並進(x)
            target_velocity[1],                 // 目標並進(y)
            target_velocity[2],                 // 目標並進(z)
        };

        return mix_to_thrusters(u);
    }
};

}  // namespace sinsei_umiusi_control::controller::logic::attitude

#endif  // SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_FEED_BACK_HPP

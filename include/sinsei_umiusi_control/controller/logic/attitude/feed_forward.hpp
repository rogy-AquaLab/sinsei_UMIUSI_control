#ifndef SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_FEED_FORWARD_HPP
#define SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_FEED_FORWARD_HPP

#include <Eigen/Core>

#include "sinsei_umiusi_control/controller/attitude_controller.hpp"
#include "sinsei_umiusi_control/controller/logic/attitude/data_conversion.hpp"
#include "sinsei_umiusi_control/controller/logic/attitude/mixer.hpp"

namespace sinsei_umiusi_control::controller::logic::attitude {

// IMUを使わず、目標姿勢をそのままモーメント要求とみなすオープンループ制御
class FeedForward : public AttitudeController::Logic {
  private:
    // 目標姿勢(クオータニオンのベクトル部、≒ロール・ピッチの傾き量)からモーメント要求への変換ゲイン
    static constexpr double K_ATTITUDE = 1.0;
    // 目標ヨーレートからモーメント要求への変換ゲイン
    static constexpr double K_YAW_RATE = 1.0;

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
        const auto target_attitude = to_eigen_quaternion(input.cmd.target_attitude);
        const auto target_velocity = to_eigen_vector(input.cmd.target_velocity);

        const auto u = Eigen::Vector<double, 6>{
            K_ATTITUDE * target_attitude.x(),                  // 目標ロール
            K_ATTITUDE * target_attitude.y(),                  // 目標ピッチ
            K_YAW_RATE * input.cmd.target_attitude.yaw_rate,   // 目標ヨーレート
            target_velocity[0],                                // 目標並進(x)
            target_velocity[1],                                // 目標並進(y)
            target_velocity[2],                                // 目標並進(z)
        };

        return mix_to_thrusters(u);
    }
};

}  // namespace sinsei_umiusi_control::controller::logic::attitude

#endif  // SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_FEED_FORWARD_HPP

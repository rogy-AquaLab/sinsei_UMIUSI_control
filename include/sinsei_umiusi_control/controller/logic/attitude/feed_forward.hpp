#ifndef SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_FEED_FORWARD_HPP
#define SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_FEED_FORWARD_HPP

#include <Eigen/Core>

#include "sinsei_umiusi_control/controller/attitude_controller.hpp"
#include "sinsei_umiusi_control/controller/logic/attitude/data_conversion.hpp"
#include "sinsei_umiusi_control/controller/logic/attitude/mixer.hpp"

namespace sinsei_umiusi_control::controller::logic::attitude {

struct FeedForwardGains {
    // 目標姿勢(クオータニオンのベクトル部、≒ロール・ピッチの傾き量)から
    // モーメント要求への変換ゲイン。UIは角度をquaternionへ変換するため、
    // 小角時にはベクトル部が従来操作量の約半分になる。
    double k_attitude{2.0};
    // UIの目標yawレート範囲(±1 rad/s)を従来のFF yaw操作量(±0.2)へ変換する。
    double k_yaw_rate{0.2};
};

// IMUを使わず、目標姿勢をそのままモーメント要求とみなすオープンループ制御
class FeedForward : public AttitudeController::Logic {
  private:
    FeedForwardGains gains;
    MixerParameters mixer_parameters;
    MixerState mixer_state;

  public:
    FeedForward(FeedForwardGains gains, MixerParameters mixer_parameters)
    : gains(gains), mixer_parameters(mixer_parameters) {}

    auto control_mode() const -> logic::ControlMode override {
        return logic::ControlMode::FeedForward;
    }

    auto init(
        double /*time*/, const AttitudeController::Input & /*input*/,
        const AttitudeController::Output & output) -> AttitudeController::Output override {
        return output;
    }

    auto update(double /*time*/, double duration, const AttitudeController::Input & input)
        -> AttitudeController::Output override {
        const auto target_attitude = to_eigen_quaternion(input.cmd.target_attitude);
        const auto target_velocity = to_eigen_vector(input.cmd.target_velocity);

        const auto u = Eigen::Vector<double, 6>{
            this->gains.k_attitude * target_attitude.x(),                 // 目標ロール
            this->gains.k_attitude * target_attitude.y(),                 // 目標ピッチ
            this->gains.k_yaw_rate * input.cmd.target_attitude.yaw_rate,  // 目標ヨーレート
            target_velocity[0],                                           // 目標並進(x)
            target_velocity[1],                                           // 目標並進(y)
            target_velocity[2],                                           // 目標並進(z)
        };

        return mix_to_thrusters(
            u, input.state.servo_estimated_angles, input.state.servo_max_angular_velocities,
            duration, this->mixer_parameters, this->mixer_state);
    }
};

}  // namespace sinsei_umiusi_control::controller::logic::attitude

#endif  // SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_FEED_FORWARD_HPP

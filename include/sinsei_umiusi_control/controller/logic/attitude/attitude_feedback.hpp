#ifndef SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_ATTITUDE_FEEDBACK_HPP
#define SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_ATTITUDE_FEEDBACK_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <optional>

namespace sinsei_umiusi_control::controller::logic::attitude {

struct AttitudeFeedbackGains {
    double kp_roll{1.0};
    double kp_pitch{1.0};
    double kd_roll{0.35};
    double kd_pitch{0.35};
    double kp_yaw_rate{1.0};
};

// Roll/pitch は body-up の向きだけを合わせる reduced-attitude 制御、yaw はレート制御。
// 目標 quaternion の yaw は意図的に無視する。
class AttitudeFeedback {
  private:
    AttitudeFeedbackGains gains;

    static auto valid_quaternion(const Eigen::Quaterniond & quaternion) -> bool {
        return quaternion.coeffs().allFinite() && quaternion.squaredNorm() > 1e-12;
    }

  public:
    explicit AttitudeFeedback(AttitudeFeedbackGains gains = {}) : gains(gains) {}

    auto moment(
        Eigen::Quaterniond target_attitude, Eigen::Quaterniond current_attitude,
        const Eigen::Vector3d & angular_velocity, double target_yaw_rate) const
        -> std::optional<Eigen::Vector3d> {
        if (!valid_quaternion(target_attitude) || !valid_quaternion(current_attitude) ||
            !angular_velocity.allFinite() || !std::isfinite(target_yaw_rate)) {
            return std::nullopt;
        }

        target_attitude.normalize();
        current_attitude.normalize();

        const auto current_rotation = current_attitude.toRotationMatrix();
        const auto current_up_world = current_rotation * Eigen::Vector3d::UnitZ();
        const auto target_up_world =
            target_attitude.toRotationMatrix() * Eigen::Vector3d::UnitZ();

        // 現在の body-up を目標へ向ける最短回転軸を body frame へ戻す。
        const auto tilt_error_body =
            current_rotation.transpose() * current_up_world.cross(target_up_world);

        return Eigen::Vector3d{
            this->gains.kp_roll * tilt_error_body.x() -
                this->gains.kd_roll * angular_velocity.x(),
            this->gains.kp_pitch * tilt_error_body.y() -
                this->gains.kd_pitch * angular_velocity.y(),
            this->gains.kp_yaw_rate * (target_yaw_rate - angular_velocity.z()),
        };
    }
};

}  // namespace sinsei_umiusi_control::controller::logic::attitude

#endif  // SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_ATTITUDE_FEEDBACK_HPP

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
        if (!quaternion.coeffs().allFinite()) {
            return false;
        }
        // geometry_msgs/Quaternion は単位 quaternion が前提。ほぼゼロのI2C異常値を
        // normalize()すると任意の姿勢として扱われるため、単位長から大きく外れた値は拒否する。
        constexpr auto MIN_SQUARED_NORM = 0.5;
        constexpr auto MAX_SQUARED_NORM = 1.5;
        const auto squared_norm = quaternion.squaredNorm();
        return squared_norm >= MIN_SQUARED_NORM && squared_norm <= MAX_SQUARED_NORM;
    }

    // Eigen の 3x3 回転行列式は GCC 13/aarch64 の最適化時に
    // -Wmaybe-uninitialized を誤検出するため、必要な演算だけを明示する。
    static auto body_up_in_world(const Eigen::Quaterniond & attitude) -> Eigen::Vector3d {
        const auto w = attitude.w();
        const auto x = attitude.x();
        const auto y = attitude.y();
        const auto z = attitude.z();
        return {
            2.0 * (x * z + w * y),
            2.0 * (y * z - w * x),
            1.0 - 2.0 * (x * x + y * y),
        };
    }

    static auto rotate_world_to_body(
        const Eigen::Quaterniond & attitude, const Eigen::Vector3d & vector) -> Eigen::Vector3d {
        const auto w = attitude.w();
        const auto x = attitude.x();
        const auto y = attitude.y();
        const auto z = attitude.z();
        return {
            (1.0 - 2.0 * (y * y + z * z)) * vector.x() + 2.0 * (x * y + w * z) * vector.y() +
                2.0 * (x * z - w * y) * vector.z(),
            2.0 * (x * y - w * z) * vector.x() + (1.0 - 2.0 * (x * x + z * z)) * vector.y() +
                2.0 * (y * z + w * x) * vector.z(),
            2.0 * (x * z + w * y) * vector.x() + 2.0 * (y * z - w * x) * vector.y() +
                (1.0 - 2.0 * (x * x + y * y)) * vector.z(),
        };
    }

  public:
    explicit AttitudeFeedback(AttitudeFeedbackGains gains = {}) : gains(gains) {}

    auto moment(
        Eigen::Quaterniond target_attitude, Eigen::Quaterniond current_attitude,
        const Eigen::Vector3d & angular_velocity,
        double target_yaw_rate) const -> std::optional<Eigen::Vector3d> {
        if (!valid_quaternion(target_attitude) || !valid_quaternion(current_attitude) ||
            !angular_velocity.allFinite() || !std::isfinite(target_yaw_rate)) {
            return std::nullopt;
        }

        target_attitude.normalize();
        current_attitude.normalize();

        const auto current_up_world = body_up_in_world(current_attitude);
        const auto target_up_world = body_up_in_world(target_attitude);

        // 現在の body-up を目標へ向ける最短回転軸を body frame へ戻す。
        const Eigen::Vector3d tilt_error_world{
            current_up_world.y() * target_up_world.z() - current_up_world.z() * target_up_world.y(),
            current_up_world.z() * target_up_world.x() - current_up_world.x() * target_up_world.z(),
            current_up_world.x() * target_up_world.y() - current_up_world.y() * target_up_world.x(),
        };
        const auto tilt_error_body = rotate_world_to_body(current_attitude, tilt_error_world);

        return Eigen::Vector3d{
            this->gains.kp_roll * tilt_error_body.x() - this->gains.kd_roll * angular_velocity.x(),
            this->gains.kp_pitch * tilt_error_body.y() -
                this->gains.kd_pitch * angular_velocity.y(),
            this->gains.kp_yaw_rate * (target_yaw_rate - angular_velocity.z()),
        };
    }
};

}  // namespace sinsei_umiusi_control::controller::logic::attitude

#endif  // SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_ATTITUDE_FEEDBACK_HPP

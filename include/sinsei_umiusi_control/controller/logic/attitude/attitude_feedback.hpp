#ifndef SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_ATTITUDE_FEEDBACK_HPP
#define SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_ATTITUDE_FEEDBACK_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <optional>

namespace sinsei_umiusi_control::controller::logic::attitude {

struct AttitudeFeedbackGains {
    double kp_roll{1.0};
    double kp_pitch{1.0};
    double kd_roll{0.35};
    double kd_pitch{0.35};
    double kp_yaw_rate{1.0};
    // 浮心オフセットやサーボ中立点のずれ等、モデル誤差由来の定常的な傾きを消すための積分項。
    // 既定 0.0 は従来通り（PD相当）。プールで定常的な傾きが見えた場合にのみ上げる。
    double ki_roll{0.0};
    double ki_pitch{0.0};
    double i_max{0.2};  // 積分項自体をクランプする簡易アンチワインドアップ

    // --- 方位保持 (hold_yaw) 用。hold_yaw = false のときは一切効かない ---
    // 方位誤差 [rad] を目標ヨーレート [rad/s] へ直すゲイン。レート制御の外側に被せる。
    // **実機未検証の値**。プールで振ってから確定すること。
    double kp_yaw_hold{1.0};
    // 方位誤差がこれを超えたら「IMUの方位が飛んだ」とみなし、追いかけずにラッチし直す [rad]。
    // 実測では跳躍が 169 deg、通常の追従誤差が 6.7〜29.2 deg なので、その間に置く。
    double yaw_hold_relatch_error{boost::math::constants::half_pi<double>()};  // 90 deg
};

// Roll/pitch は body-up の向きだけを合わせる reduced-attitude 制御、yaw はレート制御。
// 目標 quaternion の yaw は意図的に無視する。
//
// `hold_yaw` が true の間だけ、レート制御の外側に方位保持ループが乗る。保持する方位は
// **false -> true のエッジで実測方位をラッチ**して決める。「ヨーレートがほぼ 0 なら保持」
// のような推定はしない — 閾値と継続時間というノブが増えるうえ、閾値の境目でラッチし直して
// 方位が少しずつずれるため、「流されても気付けない」という保持を入れたい理由そのものが壊れる。
class AttitudeFeedback {
  private:
    AttitudeFeedbackGains gains;
    // moment() は const だが、積分項は制御周期をまたいで蓄積する必要があるため mutable にする。
    mutable double i_err_roll{0.0};
    mutable double i_err_pitch{0.0};
    // 方位保持の状態。同じ理由で mutable。
    mutable bool yaw_latched{false};
    mutable double latched_heading{0.0};
    // 直近の update でラッチし直したか（IMUの方位が飛んだことの検出）。呼び出し側がログに使う。
    mutable bool yaw_relatched{false};

    static constexpr auto PI = boost::math::constants::pi<double>();

    // [-pi, pi) へ畳む。
    static auto wrap_to_pi(double angle) -> double {
        constexpr auto TWO_PI = 2.0 * PI;
        angle = std::fmod(angle + PI, TWO_PI);
        if (angle < 0.0) {
            angle += TWO_PI;
        }
        return angle - PI;
    }

    // 重力基準 world 系での鉛直まわりの回転角 = BNO055 の磁気基準の方位。
    // roll/pitch は重力基準で壊れないが、**この軸だけは磁気外乱で飛ぶ**（known_issues A-1）。
    static auto heading_of(const Eigen::Quaterniond & attitude) -> double {
        const auto w = attitude.w();
        const auto x = attitude.x();
        const auto y = attitude.y();
        const auto z = attitude.z();
        return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    }

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

    // disarm やモード切替で状態を持ち越さないための初期化。
    void reset() const {
        this->i_err_roll = 0.0;
        this->i_err_pitch = 0.0;
        this->yaw_latched = false;
        this->latched_heading = 0.0;
        this->yaw_relatched = false;
    }

    // 直近の moment() で方位をラッチし直したか。IMUの方位が飛んだ合図なので、呼び出し側は
    // これを見て警告を出す。**目標方位は飛ぶ前の基準で与えられていたので、飛んだあとは
    // 保持している方位の意味が変わっている**（known_issues A-1 と同じ話）。
    auto yaw_was_relatched() const -> bool { return this->yaw_relatched; }
    auto holding_yaw() const -> bool { return this->yaw_latched; }
    auto latched_yaw() const -> double { return this->latched_heading; }

    // duration (制御周期 [s]) は既定 0.0 のままなら積分項が蓄積しないため、既存呼び出しの
    // 挙動は変えない。hold_yaw も既定 false なので、既存呼び出しは従来どおりレート制御。
    auto moment(
        Eigen::Quaterniond target_attitude, Eigen::Quaterniond current_attitude,
        const Eigen::Vector3d & angular_velocity, double target_yaw_rate, double duration = 0.0,
        bool hold_yaw = false) const -> std::optional<Eigen::Vector3d> {
        if (!valid_quaternion(target_attitude) || !valid_quaternion(current_attitude) ||
            !angular_velocity.allFinite() || !std::isfinite(target_yaw_rate) ||
            !std::isfinite(duration) || duration < 0.0) {
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

        this->i_err_roll = std::clamp(
            this->i_err_roll + tilt_error_body.x() * duration, -this->gains.i_max,
            this->gains.i_max);
        this->i_err_pitch = std::clamp(
            this->i_err_pitch + tilt_error_body.y() * duration, -this->gains.i_max,
            this->gains.i_max);

        const auto commanded_yaw_rate =
            this->update_yaw_hold(current_attitude, target_yaw_rate, duration, hold_yaw);

        return Eigen::Vector3d{
            this->gains.kp_roll * tilt_error_body.x() - this->gains.kd_roll * angular_velocity.x() +
                this->gains.ki_roll * this->i_err_roll,
            this->gains.kp_pitch * tilt_error_body.y() -
                this->gains.kd_pitch * angular_velocity.y() +
                this->gains.ki_pitch * this->i_err_pitch,
            this->gains.kp_yaw_rate * (commanded_yaw_rate - angular_velocity.z()),
        };
    }

  private:
    // 方位保持のラッチを進め、レート制御へ渡す目標ヨーレートを返す。
    // hold_yaw が false の間はラッチを捨て、target_yaw_rate をそのまま通す（従来の挙動）。
    auto update_yaw_hold(
        const Eigen::Quaterniond & current_attitude, double target_yaw_rate, double duration,
        bool hold_yaw) const -> double {
        this->yaw_relatched = false;

        if (!hold_yaw) {
            this->yaw_latched = false;
            return target_yaw_rate;
        }

        const auto heading = heading_of(current_attitude);

        if (!this->yaw_latched) {
            // false -> true のエッジ。**いまの方位**を保持対象にする。
            this->latched_heading = heading;
            this->yaw_latched = true;
            return target_yaw_rate;
        }

        // 保持中の yaw_rate は「保持したまま向きを変える」指令として扱う。ラッチ値を同じだけ
        // 動かし、同時にフィードフォワードとしても渡す。こうすると保持とレートが別モードでは
        // なく連続になり、小さな修正のたびに hold_yaw をトグルしなくてよい。
        this->latched_heading = wrap_to_pi(this->latched_heading + target_yaw_rate * duration);

        const auto error = wrap_to_pi(this->latched_heading - heading);
        if (std::abs(error) > this->gains.yaw_hold_relatch_error) {
            // 追いかけない。**追いかけると跳躍がそのまま「180度回れ」という指令に化ける。**
            this->latched_heading = heading;
            this->yaw_relatched = true;
            return target_yaw_rate;
        }

        return target_yaw_rate + this->gains.kp_yaw_hold * error;
    }
};

}  // namespace sinsei_umiusi_control::controller::logic::attitude

#endif  // SINSEI_UMIUSI_CONTROL_CONTROLLER_LOGIC_ATTITUDE_ATTITUDE_FEEDBACK_HPP

#ifndef SINSEI_UMIUSI_CONTROL_CONTROLLER_THRUSTER_LIMITS_HPP
#define SINSEI_UMIUSI_CONTROL_CONTROLLER_THRUSTER_LIMITS_HPP

#include <algorithm>

namespace sinsei_umiusi_control::controller {

/// ハードウェア境界の歯止め。指令の出所によらず必ず通す。
///
/// 置くのはステートレスな範囲の強制だけ:
///   * スルーレート制限は置かない。持ち主は指令を出す側で、素通し経路に状態を持つ制限を
///     挟むと二重に掛かる。core 経路は logic::thruster::LinearAcceleration が持つ
///   * is_forward / duty_per_thrust も置かない。あれは推力[N] -> Duty の換算であって
///     範囲の強制ではない。直接指令はすでに Duty で届くので掛け直すと二重換算になる
///
/// 冪等であること: output.state は書き換えず、output.cmd を作るときにだけ通す。
/// 書き戻すと次の周期に重ね掛けされる (servo_sign = -1 で符号が周期ごとに反転する)。
///
/// 経緯は sinsei_UMIUSI_autonomy の docs/known_issues.md B-12。
class ThrusterLimits {
  public:
    /// サーボ角のハード上限 [deg]。範囲外は clamp ではなく CAN フレームの送信自体が
    /// 失敗する (can::VescModel::make_servo_angle_frame)。
    static constexpr double SERVO_ANGLE_LIMIT_DEG = 90.0;

    ThrusterLimits() = default;
    ThrusterLimits(double max_duty, double servo_sign)  // NOLINT
    : max_duty(max_duty), servo_sign(servo_sign < 0.0 ? -1.0 : 1.0) {}

    auto apply_duty(double duty) const -> double {
        return std::clamp(duty, -this->max_duty, this->max_duty);
    }

    /// 回転センスを掛けて ±90 に収める (範囲が対称なので順序は結果に影響しない)。
    auto apply_servo_angle(double deg) const -> double {
        return std::clamp(this->servo_sign * deg, -SERVO_ANGLE_LIMIT_DEG, SERVO_ANGLE_LIMIT_DEG);
    }

    /// クランプが効いたか。較正は生の指令を出す前提なので、上限に当たったことに
    /// 気付けないと推力カーブの上端を誤って測る。
    auto duty_was_clamped(double duty) const -> bool {
        return std::abs(duty) > this->max_duty;
    }
    auto servo_angle_was_clamped(double deg) const -> bool {
        return std::abs(deg) > SERVO_ANGLE_LIMIT_DEG;
    }

  private:
    double max_duty{0.0};
    double servo_sign{1.0};
};

}  // namespace sinsei_umiusi_control::controller

#endif  // SINSEI_UMIUSI_CONTROL_CONTROLLER_THRUSTER_LIMITS_HPP

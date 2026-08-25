#ifndef SINSEI_UMIUSI_CONTROL_CONTROLLER_THRUSTER_LIMITS_HPP
#define SINSEI_UMIUSI_CONTROL_CONTROLLER_THRUSTER_LIMITS_HPP

#include <algorithm>

namespace sinsei_umiusi_control::controller {

/// ハードウェア境界の歯止め。**指令の出所によらず必ず通す**もの。
///
/// `cmd/direct/thruster_controller/output_*` にパブリッシャが居ると
/// `thruster_controller.cpp` が `logic->update()` ごとスキップするため、直接指令の経路には
/// Duty上限もサーボ角の範囲チェックも無かった。coreの電源/モード調停をバイパスするのは
/// 意図どおりだが、**ハードを壊しうる範囲外の値まで素通しになるのは巻き添え**だった
/// (`sinsei_UMIUSI_autonomy` の `docs/known_issues.md` B-12)。
///
/// ここに置くのは **ステートレスな範囲の強制だけ**。意図的にそう決めている:
///
/// * **スルーレート制限はここに置かない。** あれはハードの保護ではなく
///   「方策が学習したプラント (シミュレータ) を実機で再現する」ための平滑化で、
///   持ち主は指令を出す側 (`sinsei_UMIUSI_autonomy` の `thruster_limits.py`)。
///   状態を持つ制限を素通し経路に挟むと、(a) 生の指令をラッチしないと制限後の値に
///   制限が重ね掛けされる (b) 実効レートが publisher のレートに依存する
///   (c) 較正ツールが生の指令を出せなくなる、という三重の問題が出る。
///   core 経路のスルーレートは従来どおり `logic::thruster::LinearAcceleration` が持つ。
/// * `is_forward` と `duty_per_thrust` もここには無い。あれは「推力[N] → Duty」の
///   **換算**であって範囲の強制ではない。直接指令はすでにDutyで届くので、掛け直すと
///   二重換算になる (`sinsei_UMIUSI_autonomy` 側のアロケーションが、プラントの二次カーブを
///   逆写像でプリワープした Duty を出している)。
///
/// ステートレスなので**冪等**であることが重要: `output.state` を書き換えず、
/// ハードへ出す `output.cmd` を作るときにだけ通す。
class ThrusterLimits {
  public:
    /// サーボ角のハード上限 [deg]。VESCは -90.0 ~ 90.0 を 0.0 ~ 1.0 に写す
    /// (`can::VescModel::make_servo_angle_frame`)。範囲外はクランプではなく
    /// **CANフレームの送信そのものが失敗**していたので、ここで収めてから出す。
    static constexpr double SERVO_ANGLE_LIMIT_DEG = 90.0;

    ThrusterLimits() = default;
    ThrusterLimits(double max_duty, double servo_sign)  // NOLINT
    : max_duty(max_duty), servo_sign(servo_sign < 0.0 ? -1.0 : 1.0) {}

    /// Dutyを絶対値上限に収める。
    auto apply_duty(double duty) const -> double {
        return std::clamp(duty, -this->max_duty, this->max_duty);
    }

    /// サーボ角[deg]に取り付けの回転センスを掛け、ハードの ±90 に収める。
    /// (範囲が対称なのでクランプと符号の順序は結果に影響しない。)
    auto apply_servo_angle(double deg) const -> double {
        return std::clamp(this->servo_sign * deg, -SERVO_ANGLE_LIMIT_DEG, SERVO_ANGLE_LIMIT_DEG);
    }

    /// 指令が範囲外で、実際にクランプが効いたか。**黙って値を削らない**ための問い合わせ。
    /// 較正 (`thruster_cmd.py`) は生の指令を出す前提なので、上限に当たったことに
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

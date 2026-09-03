#include "sinsei_umiusi_control/controller/thruster_limits.hpp"

#include <gtest/gtest.h>

namespace controller = sinsei_umiusi_control::controller;

namespace {

// ステートレスであることが設計の要。thruster_controller は永続メンバ output を持ち、
// メッセージが届かなかった周期にも update_and_write_commands が走るため、冪等でないと壊れる。

TEST(ThrusterLimits, DutyIsClampedToMaxDuty) {
    const auto limits = controller::ThrusterLimits{0.5, 1.0};
    EXPECT_DOUBLE_EQ(limits.apply_duty(1.0), 0.5);
    EXPECT_DOUBLE_EQ(limits.apply_duty(-1.0), -0.5);
    EXPECT_DOUBLE_EQ(limits.apply_duty(0.3), 0.3);
}

TEST(ThrusterLimits, ZeroMaxDutyProducesNoOutput) {
    EXPECT_DOUBLE_EQ(controller::ThrusterLimits(0.0, 1.0).apply_duty(1.0), 0.0);
}

TEST(ThrusterLimits, ServoAngleIsClampedToHardwareRange) {
    const auto limits = controller::ThrusterLimits{0.5, 1.0};
    // 範囲外は以前クランプされず、`make_servo_angle_frame` が CAN フレームの送信ごと
    // 失敗していた (歯止めがどこにも無かった)
    EXPECT_DOUBLE_EQ(limits.apply_servo_angle(120.0), 90.0);
    EXPECT_DOUBLE_EQ(limits.apply_servo_angle(-120.0), -90.0);
    EXPECT_DOUBLE_EQ(limits.apply_servo_angle(45.0), 45.0);
}

TEST(ThrusterLimits, ServoSignFlipsRotationSense) {
    const auto limits = controller::ThrusterLimits{0.5, -1.0};
    EXPECT_DOUBLE_EQ(limits.apply_servo_angle(45.0), -45.0);
    EXPECT_DOUBLE_EQ(limits.apply_servo_angle(-120.0), 90.0);
}

TEST(ThrusterLimits, ServoSignIsNormalisedToPlusOrMinusOne) {
    // 0.0 や中途半端な値でスケールされてしまうと、サーボ角が黙って縮む
    EXPECT_DOUBLE_EQ(controller::ThrusterLimits(0.5, 0.0).apply_servo_angle(45.0), 45.0);
    EXPECT_DOUBLE_EQ(controller::ThrusterLimits(0.5, 0.5).apply_servo_angle(45.0), 45.0);
    EXPECT_DOUBLE_EQ(controller::ThrusterLimits(0.5, -0.5).apply_servo_angle(45.0), -45.0);
}

// --- ステートレス性 / 冪等性 ---
// これが崩れると、直接指令の経路で **メッセージが届かなかった周期に制限が重ね掛けされ**、
// `servo_sign = -1` のとき角度の符号が周期ごとに反転してサーボがチャタリングする。

TEST(ThrusterLimits, IsIdempotentSoRepeatedApplicationIsSafe) {
    for (const auto sign : {1.0, -1.0}) {
        const auto limits = controller::ThrusterLimits{0.5, sign};
        for (const auto raw : {0.0, 0.3, 0.7, -0.7, 1.0}) {
            const auto once = limits.apply_duty(raw);
            EXPECT_DOUBLE_EQ(limits.apply_duty(once), once) << "duty " << raw;
        }
        for (const auto raw : {0.0, 45.0, 120.0, -120.0}) {
            const auto once = limits.apply_servo_angle(raw);
            // **符号は 1 回しか掛けてはいけない**ので、冪等性は「生の値から作り直す」
            // 前提でのみ成り立つ。ここでは出力をもう一度通しても角度が動かないことを
            // 要求する — 掛け直すと sign が二度効いて反転するため、sign=-1 では
            // `apply(apply(x)) != apply(x)` になる。これがコントローラ側で
            // `output.state` を書き換えてはいけない理由。
            if (sign > 0.0) {
                EXPECT_DOUBLE_EQ(limits.apply_servo_angle(once), once) << "angle " << raw;
            } else {
                EXPECT_DOUBLE_EQ(limits.apply_servo_angle(once), -once) << "angle " << raw;
            }
        }
    }
}

TEST(ThrusterLimits, RepeatedCyclesOnALatchedCommandAreStable) {
    // コントローラの周期を模す: 指令は 1 回だけ届き、以後メッセージが来ない。
    // **毎周期 生の指令から作り直す**ので、出力は一定でなければならない。
    const auto limits = controller::ThrusterLimits{0.5, -1.0};
    constexpr auto RAW_DUTY = 0.7;      // 上限超え
    constexpr auto RAW_ANGLE = 45.0;
    for (auto i = 0; i < 20; ++i) {
        EXPECT_DOUBLE_EQ(limits.apply_duty(RAW_DUTY), 0.5) << "cycle " << i;
        EXPECT_DOUBLE_EQ(limits.apply_servo_angle(RAW_ANGLE), -45.0) << "cycle " << i;
    }
}

TEST(ThrusterLimits, ClampIsReportedSoItIsNeverSilent) {
    // 較正 (`thruster_cmd.py`) は生の指令を出す前提。上限に当たったことに気付けないと
    // 推力カーブの上端を誤って測る。
    const auto limits = controller::ThrusterLimits{0.5, 1.0};
    EXPECT_TRUE(limits.duty_was_clamped(0.6));
    EXPECT_FALSE(limits.duty_was_clamped(0.5));
    EXPECT_FALSE(limits.duty_was_clamped(-0.4));
    EXPECT_TRUE(limits.servo_angle_was_clamped(-91.0));
    EXPECT_FALSE(limits.servo_angle_was_clamped(90.0));
}

}  // namespace

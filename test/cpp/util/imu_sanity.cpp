#include "sinsei_umiusi_control/util/imu_sanity.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>

namespace util = sinsei_umiusi_control::util;

namespace {

constexpr std::array<double, 4> LEVEL{1.0, 0.0, 0.0, 0.0};  // (w, x, y, z)
constexpr std::array<double, 3> STILL{0.0, 0.0, 0.0};

auto about_z(double deg) -> std::array<double, 4> {
    const auto h = deg * M_PI / 180.0 / 2.0;
    return {std::cos(h), 0.0, 0.0, std::sin(h)};
}

}  // namespace

// --- 数値として使えない値は enforce に関係なく捨てる ---------------------------
// 閾値の問題ではなく正規化が 0 除算になるため。実機で最も多い化け方でもある
// (autonomy known_issues A-1、2026-09-06 のベンチ 915 秒で 7 件すべてこの形)。

TEST(ImuSanity, ExactZeroNormIsAlwaysHeldEvenWhenNotEnforcing) {
    auto s = util::ImuSanity{};  // 既定は enforce=false
    s.update(LEVEL, STILL);

    const auto r = s.update({0.0, 0.0, 0.0, 0.0}, STILL);
    EXPECT_EQ(r.reason, util::ImuSanity::Reason::Unnormalizable);
    ASSERT_TRUE(r.sample);
    EXPECT_DOUBLE_EQ(r.sample->quat[0], 1.0) << "直前の有効値が返っていない";
    EXPECT_EQ(s.rejected(), 1u);
}

// 実機で実際に出る化けは MIN_NORM (1e-6) を 2 桁上回るので Unnormalizable ではなく
// BadNorm に落ちる (2026-09-06 のベンチで |q| は 1.2e-4 〜 2.23)。正規化すると
// 「もっともらしい単位クォータニオン」になり、通すと下流で気付けない
// (この例は 120 deg 回転)。BadNorm は絶対判定なので既定で棄却する。
TEST(ImuSanity, RealWorldCorruptionIsRejectedByDefault) {
    auto s = util::ImuSanity{};
    s.update(LEVEL, STILL);

    // 実機で観測した値: 全成分が -0.0001 付近、|q|=0.0002
    const auto r = s.update({-0.0001, -0.0001, -0.0001, -0.0001}, STILL);
    EXPECT_EQ(r.reason, util::ImuSanity::Reason::BadNorm);
    EXPECT_TRUE(r.held);
    ASSERT_TRUE(r.sample);
    EXPECT_DOUBLE_EQ(r.sample->quat[0], 1.0) << "直前の有効値が返るはず";
    EXPECT_EQ(s.rejected(), 1u);
}

TEST(ImuSanity, NonFiniteIsAlwaysHeld) {
    auto s = util::ImuSanity{};
    s.update(LEVEL, STILL);
    const auto r = s.update({std::nan(""), 0.0, 0.0, 0.0}, STILL);
    EXPECT_EQ(r.reason, util::ImuSanity::Reason::NotFinite);
    EXPECT_DOUBLE_EQ(r.sample->quat[0], 1.0);
}

TEST(ImuSanity, NoValidSampleYetReturnsNothing) {
    auto s = util::ImuSanity{};
    const auto r = s.update({0.0, 0.0, 0.0, 0.0}, STILL);
    EXPECT_FALSE(r.sample);
}

// --- 棄却を切ると検出だけして通す ---------------------------------------------
// 閾値を決め直すためのデータ収集で使う。

TEST(ImuSanity, DetectsButPassesWhenAbsoluteEnforcementIsOff) {
    auto opt = util::ImuSanity::Options{};
    opt.enforce_absolute = false;
    auto s = util::ImuSanity{opt};
    s.update(LEVEL, STILL);
    // 実機で観測したノルム異常 |q|=2.2306 (同じ値が 10 回繰り返した)
    const auto r = s.update({2.2306, 0.0, 0.0, 0.0}, STILL);
    EXPECT_EQ(r.reason, util::ImuSanity::Reason::BadNorm);
    EXPECT_FALSE(r.held);
    EXPECT_EQ(s.rejected(), 0u);
    EXPECT_EQ(s.flagged(), 1u);
}

TEST(ImuSanity, DiscardsWhenEnforcing) {
    auto opt = util::ImuSanity::Options{};
    opt.enforce_step = true;
    auto s = util::ImuSanity{opt};
    s.update(LEVEL, STILL);
    const auto r = s.update(about_z(10.0), {20.0, 0.0, 0.0});
    EXPECT_EQ(r.reason, util::ImuSanity::Reason::GyroOverLimit);
    EXPECT_EQ(s.rejected(), 1u);
    EXPECT_DOUBLE_EQ(r.sample->quat[0], 1.0) << "直前の有効値が返るはず";
}

// --- 角速度 ------------------------------------------------------------------

TEST(ImuSanity, FullScaleGyroIsCalledOutInTheMessage) {
    auto s = util::ImuSanity{};
    // int16 フルスケール (32767/16 = 2047.9 deg/s = 35.74 rad/s)
    const auto r = s.update(LEVEL, {35.74, 35.74, 35.74});
    EXPECT_EQ(r.reason, util::ImuSanity::Reason::GyroOverLimit);
    EXPECT_NE(r.detail.find("フルスケール"), std::string::npos);
}

TEST(ImuSanity, RealMotionPasses) {
    auto s = util::ImuSanity{};
    // 実機で手で振ったときの最大は 4.6 rad/s (閾値 10 に対し 2.2 倍の余裕)
    const auto r = s.update(LEVEL, {4.6, 0.0, 0.0});
    EXPECT_EQ(r.reason, util::ImuSanity::Reason::None);
}

// --- 姿勢の跳躍 ---------------------------------------------------------------

TEST(ImuSanity, SignFlipIsNotAJump) {
    auto s = util::ImuSanity{};
    s.update(LEVEL, STILL);
    // q と -q は同じ姿勢。符号の曖昧さを跳躍と誤判定してはいけない
    const auto r = s.update({-1.0, 0.0, 0.0, 0.0}, STILL);
    EXPECT_EQ(r.reason, util::ImuSanity::Reason::None);
}

TEST(ImuSanity, NormalStepPassesAndLargeStepIsFlagged) {
    auto s = util::ImuSanity{};
    s.update(LEVEL, STILL);
    // 実機で手で振ったときの 1 サンプル最大は 6.1 deg (閾値 30 に対し 4.9 倍の余裕)
    EXPECT_EQ(s.update(about_z(6.1), STILL).reason, util::ImuSanity::Reason::None);
    EXPECT_EQ(s.update(about_z(80.0), STILL).reason, util::ImuSanity::Reason::AttitudeStep);
}

// --- stale による再同期 -------------------------------------------------------
// 姿勢基準そのものが飛ぶと、飛ぶ前の値と比べ続ける限り永久に復帰できない。
// known_issues A-1 では実機で 144 秒 (連続 7202 回) 棄却し続けた。

TEST(ImuSanity, ResyncsAfterTheReferenceItselfJumps) {
    auto opt = util::ImuSanity::Options{};
    opt.enforce_step = true;
    opt.stale_after = 5;
    auto s = util::ImuSanity{opt};
    s.update(LEVEL, STILL);

    // 基準が 169 deg 飛び、飛んだ先で正常に追従を続ける
    const auto jumped = about_z(169.0);
    for (int i = 0; i < opt.stale_after + 1; ++i) {
        EXPECT_EQ(s.update(jumped, STILL).reason, util::ImuSanity::Reason::AttitudeStep)
            << "stale に達するまでは棄却され続ける (i=" << i << ")";
    }
    ASSERT_TRUE(s.stale());

    // stale に達したら跳躍チェックだけ解除して再同期する
    const auto r = s.update(jumped, STILL);
    EXPECT_EQ(r.reason, util::ImuSanity::Reason::None);
    EXPECT_EQ(s.resyncs(), 1u);
    EXPECT_FALSE(s.stale()) << "再同期したら stale は解けるはず";

    // 絶対値で判定できるものは stale 中でも弾き続ける
    EXPECT_TRUE(util::ImuSanity::unusable(
        s.update({0.0, 0.0, 0.0, 0.0}, STILL).reason));
}

TEST(ImuSanity, AngleBetweenAbsorbsSignAmbiguity) {
    EXPECT_NEAR(util::quat_angle_between(LEVEL, {-1.0, 0.0, 0.0, 0.0}), 0.0, 1e-9);
    EXPECT_NEAR(util::quat_angle_between(LEVEL, about_z(90.0)), M_PI / 2.0, 1e-9);
}

// --- held: 呼び出し側が生値を通してよいかの判定 -------------------------------
// enforce=false で生データを録り続けるために要る。正規化した値を publish すると
// bag から |q| の化けが見えなくなり、閾値を決め直せない。

TEST(ImuSanity, HeldIsOnlySetWhenTheSampleIsActuallyReplaced) {
    auto s = util::ImuSanity{};  // 既定: 絶対判定は棄却、相対判定は通す
    s.update(LEVEL, STILL);

    EXPECT_FALSE(s.update(about_z(1.0), STILL).held) << "正常なサンプル";
    EXPECT_TRUE(s.update({2.2306, 0.0, 0.0, 0.0}, STILL).held) << "絶対判定は棄却する";
    EXPECT_FALSE(s.update(about_z(80.0), STILL).held)
        << "相対判定は既定では通す (基準が飛ぶとロックアウトするため)";
}

// --- 絶対判定と相対判定の切り分け ---------------------------------------------
// 絶対判定は履歴に依存しないので、誤爆しても次のサンプルで復帰する。
// 相対判定だけが「最後に採用した値」との比較で、基準そのものが飛ぶと復帰できない
// (known_issues A-1 で 144 秒棄却し続けた)。棄却の既定値をここで分ける根拠。

TEST(ImuSanity, AbsoluteAndRelativeAreClassifiedAsSuch) {
    using R = util::ImuSanity::Reason;
    EXPECT_TRUE(util::ImuSanity::absolute(R::NotFinite));
    EXPECT_TRUE(util::ImuSanity::absolute(R::Unnormalizable));
    EXPECT_TRUE(util::ImuSanity::absolute(R::BadNorm));
    EXPECT_TRUE(util::ImuSanity::absolute(R::GyroOverLimit));
    EXPECT_FALSE(util::ImuSanity::absolute(R::AttitudeStep)) << "唯一の相対判定";
}

TEST(ImuSanity, AbsoluteRejectionCannotLockOut) {
    // 化けが続いても、正常なサンプルが 1 つ来れば即座に復帰する
    auto s = util::ImuSanity{};
    s.update(LEVEL, STILL);
    for (int i = 0; i < 50; ++i) {
        EXPECT_TRUE(s.update({2.2306, 0.0, 0.0, 0.0}, STILL).held);
    }
    const auto r = s.update(about_z(1.0), STILL);
    EXPECT_EQ(r.reason, util::ImuSanity::Reason::None);
    EXPECT_FALSE(r.held) << "絶対判定はロックアウトしない";
}

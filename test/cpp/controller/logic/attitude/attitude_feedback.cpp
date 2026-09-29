#include "sinsei_umiusi_control/controller/logic/attitude/attitude_feedback.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <limits>

namespace sinsei_umiusi_control::test::controller::logic::attitude {

using sinsei_umiusi_control::controller::logic::attitude::AttitudeFeedback;
using sinsei_umiusi_control::controller::logic::attitude::AttitudeFeedbackGains;

constexpr auto ANGLE = 0.2;
constexpr auto EPS = 1e-12;

TEST(AttitudeFeedbackTest, LevelAndStoppedProducesNoMoment) {
    const auto feedback = AttitudeFeedback{};
    const auto moment = feedback.moment(
        Eigen::Quaterniond::Identity(), Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(),
        0.0);

    ASSERT_TRUE(moment);
    EXPECT_TRUE(moment->isZero(EPS));
}

TEST(AttitudeFeedbackTest, PositiveRollTargetProducesPositiveRollMoment) {
    const auto feedback = AttitudeFeedback{};
    const auto target = Eigen::Quaterniond{Eigen::AngleAxisd{ANGLE, Eigen::Vector3d::UnitX()}};
    const auto moment =
        feedback.moment(target, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0.0);

    ASSERT_TRUE(moment);
    EXPECT_GT(moment->x(), 0.0);
    EXPECT_NEAR(moment->y(), 0.0, EPS);
}

TEST(AttitudeFeedbackTest, PositivePitchTargetProducesPositivePitchMoment) {
    const auto feedback = AttitudeFeedback{};
    const auto target = Eigen::Quaterniond{Eigen::AngleAxisd{ANGLE, Eigen::Vector3d::UnitY()}};
    const auto moment =
        feedback.moment(target, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0.0);

    ASSERT_TRUE(moment);
    EXPECT_NEAR(moment->x(), 0.0, EPS);
    EXPECT_GT(moment->y(), 0.0);
}

TEST(AttitudeFeedbackTest, PositiveCurrentRollProducesRestoringNegativeRollMoment) {
    const auto feedback = AttitudeFeedback{};
    const auto current = Eigen::Quaterniond{Eigen::AngleAxisd{ANGLE, Eigen::Vector3d::UnitX()}};
    const auto moment =
        feedback.moment(Eigen::Quaterniond::Identity(), current, Eigen::Vector3d::Zero(), 0.0);

    ASSERT_TRUE(moment);
    EXPECT_LT(moment->x(), 0.0);
    EXPECT_NEAR(moment->y(), 0.0, EPS);
}

TEST(AttitudeFeedbackTest, PositiveCurrentPitchProducesRestoringNegativePitchMoment) {
    const auto feedback = AttitudeFeedback{};
    const auto current = Eigen::Quaterniond{Eigen::AngleAxisd{ANGLE, Eigen::Vector3d::UnitY()}};
    const auto moment =
        feedback.moment(Eigen::Quaterniond::Identity(), current, Eigen::Vector3d::Zero(), 0.0);

    ASSERT_TRUE(moment);
    EXPECT_NEAR(moment->x(), 0.0, EPS);
    EXPECT_LT(moment->y(), 0.0);
}

TEST(AttitudeFeedbackTest, TransformsWorldTiltErrorIntoCurrentBodyFrame) {
    const auto feedback = AttitudeFeedback{};
    const auto current = Eigen::Quaterniond{Eigen::AngleAxisd{M_PI_2, Eigen::Vector3d::UnitZ()}};
    const auto target = Eigen::Quaterniond{Eigen::AngleAxisd{ANGLE, Eigen::Vector3d::UnitY()}};
    const auto moment = feedback.moment(target, current, Eigen::Vector3d::Zero(), 0.0);

    ASSERT_TRUE(moment);
    EXPECT_NEAR(moment->x(), std::sin(ANGLE), EPS);
    EXPECT_NEAR(moment->y(), 0.0, EPS);
}

TEST(AttitudeFeedbackTest, AngularVelocityDampsRollAndPitch) {
    const auto feedback = AttitudeFeedback{};
    const auto moment = feedback.moment(
        Eigen::Quaterniond::Identity(), Eigen::Quaterniond::Identity(),
        Eigen::Vector3d{1.0, -2.0, 0.0}, 0.0);

    ASSERT_TRUE(moment);
    EXPECT_NEAR(moment->x(), -0.35, EPS);
    EXPECT_NEAR(moment->y(), 0.70, EPS);
}

TEST(AttitudeFeedbackTest, YawRateUsesMeasuredRateFeedback) {
    const auto feedback = AttitudeFeedback{};
    const auto moment = feedback.moment(
        Eigen::Quaterniond::Identity(), Eigen::Quaterniond::Identity(),
        Eigen::Vector3d{0.0, 0.0, 0.25}, 0.5);

    ASSERT_TRUE(moment);
    EXPECT_NEAR(moment->z(), 0.25, EPS);
}

TEST(AttitudeFeedbackTest, IgnoresTargetAndCurrentYawForTiltControl) {
    const auto feedback = AttitudeFeedback{};
    const auto current = Eigen::Quaterniond{Eigen::AngleAxisd{1.0, Eigen::Vector3d::UnitZ()}};
    const auto target = Eigen::Quaterniond{Eigen::AngleAxisd{-0.7, Eigen::Vector3d::UnitZ()}};
    const auto moment = feedback.moment(target, current, Eigen::Vector3d::Zero(), 0.0);

    ASSERT_TRUE(moment);
    EXPECT_NEAR(moment->x(), 0.0, EPS);
    EXPECT_NEAR(moment->y(), 0.0, EPS);
}

TEST(AttitudeFeedbackTest, QuaternionSignDoesNotChangeMoment) {
    const auto feedback = AttitudeFeedback{};
    const auto target = Eigen::Quaterniond{Eigen::AngleAxisd{ANGLE, Eigen::Vector3d::UnitX()}};
    auto negative_target = target;
    negative_target.coeffs() *= -1.0;

    const auto positive =
        feedback.moment(target, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0.0);
    const auto negative = feedback.moment(
        negative_target, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0.0);

    ASSERT_TRUE(positive);
    ASSERT_TRUE(negative);
    EXPECT_TRUE(positive->isApprox(*negative, EPS));
}

TEST(AttitudeFeedbackTest, RejectsInvalidQuaternionAndNonFiniteInput) {
    const auto feedback = AttitudeFeedback{};
    const auto zero_quaternion = Eigen::Quaterniond{0.0, 0.0, 0.0, 0.0};
    const auto corrupted_imu_quaternion = Eigen::Quaterniond{
        -0.00006103515625, -0.00006103515625, -0.00006103515625, -0.00006103515625};
    const auto oversized_quaternion = Eigen::Quaterniond{2.0, 0.0, 0.0, 0.0};
    const auto nan = std::numeric_limits<double>::quiet_NaN();

    EXPECT_FALSE(feedback.moment(
        zero_quaternion, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0.0));
    EXPECT_FALSE(feedback.moment(
        Eigen::Quaterniond::Identity(), corrupted_imu_quaternion, Eigen::Vector3d::Zero(), 0.0));
    EXPECT_FALSE(feedback.moment(
        oversized_quaternion, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0.0));
    EXPECT_FALSE(feedback.moment(
        Eigen::Quaterniond::Identity(), Eigen::Quaterniond::Identity(),
        Eigen::Vector3d{nan, 0.0, 0.0}, 0.0));
}

TEST(AttitudeFeedbackTest, ZeroDurationDoesNotAccumulateIntegral) {
    auto gains = AttitudeFeedbackGains{};
    gains.ki_roll = 1.0;
    gains.ki_pitch = 1.0;
    const auto feedback = AttitudeFeedback{gains};
    const auto target = Eigen::Quaterniond{Eigen::AngleAxisd{ANGLE, Eigen::Vector3d::UnitX()}};

    for (auto i = 0; i < 5; ++i) {
        feedback.moment(target, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0.0);
    }
    const auto moment =
        feedback.moment(target, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0.0);

    ASSERT_TRUE(moment);
    EXPECT_NEAR(moment->x(), std::sin(ANGLE), EPS);
}

TEST(AttitudeFeedbackTest, PersistentTiltErrorAccumulatesIntegralOverTime) {
    auto gains = AttitudeFeedbackGains{};
    gains.ki_roll = 1.0;
    gains.ki_pitch = 1.0;
    gains.i_max = 10.0;
    const auto feedback = AttitudeFeedback{gains};
    const auto target = Eigen::Quaterniond{Eigen::AngleAxisd{ANGLE, Eigen::Vector3d::UnitX()}};
    constexpr auto DURATION = 0.1;

    // 同じ傾き誤差が続くと、積分項の寄与だけ徐々にモーメントが増える。
    const auto first =
        feedback.moment(target, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0.0, DURATION);
    const auto second =
        feedback.moment(target, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0.0, DURATION);

    ASSERT_TRUE(first);
    ASSERT_TRUE(second);
    EXPECT_GT(second->x(), first->x());
}

TEST(AttitudeFeedbackTest, IntegralTermIsClampedByIMax) {
    auto gains = AttitudeFeedbackGains{};
    gains.ki_roll = 1.0;
    gains.ki_pitch = 1.0;
    gains.i_max = 0.01;
    const auto feedback = AttitudeFeedback{gains};
    const auto target = Eigen::Quaterniond{Eigen::AngleAxisd{ANGLE, Eigen::Vector3d::UnitX()}};
    constexpr auto DURATION = 1.0;

    auto last = feedback.moment(
        target, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0.0, DURATION);
    for (auto i = 0; i < 50; ++i) {
        last = feedback.moment(
            target, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0.0, DURATION);
    }
    const auto clamped = feedback.moment(
        target, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0.0, DURATION);

    ASSERT_TRUE(last);
    ASSERT_TRUE(clamped);
    // i_max でクランプされているので、これ以上呼び出しても積分項の寄与は増えない。
    EXPECT_NEAR(last->x(), clamped->x(), EPS);
}

// --- 方位保持 (hold_yaw) ---------------------------------------------------------------------

namespace {

// 鉛直 (world +Z) まわりに `yaw` だけ回した姿勢。
auto heading_quat(double yaw) -> Eigen::Quaterniond {
    return Eigen::Quaterniond{Eigen::AngleAxisd{yaw, Eigen::Vector3d::UnitZ()}};
}

constexpr auto DT = 0.02;  // 50 Hz

}  // namespace

TEST(AttitudeFeedbackHoldYawTest, HoldFalseKeepsPlainRateControl) {
    const auto feedback = AttitudeFeedback{};
    constexpr auto TARGET_RATE = 0.3;

    // 方位が大きくずれていても、hold_yaw が false なら yaw はレート指令だけで決まる。
    const auto moment = feedback.moment(
        Eigen::Quaterniond::Identity(), heading_quat(1.0), Eigen::Vector3d::Zero(), TARGET_RATE, DT,
        false);

    ASSERT_TRUE(moment);
    EXPECT_NEAR(moment->z(), TARGET_RATE, EPS);
    EXPECT_FALSE(feedback.holding_yaw());
}

TEST(AttitudeFeedbackHoldYawTest, LatchesHeadingOnRisingEdge) {
    const auto feedback = AttitudeFeedback{};
    constexpr auto HEADING = 0.7;

    const auto first = feedback.moment(
        Eigen::Quaterniond::Identity(), heading_quat(HEADING), Eigen::Vector3d::Zero(), 0.0, DT,
        true);

    ASSERT_TRUE(first);
    EXPECT_TRUE(feedback.holding_yaw());
    EXPECT_NEAR(feedback.latched_yaw(), HEADING, 1e-9);
    // ラッチした直後は誤差 0 なので、レート指令 0 に対して yaw モーメントも 0。
    EXPECT_NEAR(first->z(), 0.0, EPS);
}

TEST(AttitudeFeedbackHoldYawTest, DriftingAwayProducesRestoringYawMoment) {
    const auto feedback = AttitudeFeedback{};
    constexpr auto HEADING = 0.0;

    feedback.moment(
        Eigen::Quaterniond::Identity(), heading_quat(HEADING), Eigen::Vector3d::Zero(), 0.0, DT,
        true);
    // ラッチした方位から +0.2 rad 流された。戻す向き (負) のモーメントが出る。
    const auto drifted = feedback.moment(
        Eigen::Quaterniond::Identity(), heading_quat(HEADING + 0.2), Eigen::Vector3d::Zero(), 0.0,
        DT, true);

    ASSERT_TRUE(drifted);
    EXPECT_LT(drifted->z(), 0.0);
    EXPECT_FALSE(feedback.yaw_was_relatched());
}

TEST(AttitudeFeedbackHoldYawTest, YawRateSlewsTheLatchedHeading) {
    const auto feedback = AttitudeFeedback{};
    constexpr auto RATE = 0.5;

    feedback.moment(
        Eigen::Quaterniond::Identity(), heading_quat(0.0), Eigen::Vector3d::Zero(), 0.0, DT, true);
    const auto before = feedback.latched_yaw();
    feedback.moment(
        Eigen::Quaterniond::Identity(), heading_quat(0.0), Eigen::Vector3d::Zero(), RATE, DT, true);

    // 保持中の yaw_rate は「保持したまま向きを変える」指令。ラッチ値が rate * dt だけ動く。
    EXPECT_NEAR(feedback.latched_yaw() - before, RATE * DT, 1e-9);
}

TEST(AttitudeFeedbackHoldYawTest, RelatchesInsteadOfChasingAnImuHeadingJump) {
    const auto feedback = AttitudeFeedback{};
    // 実機 (2026-08-21, bag `data/imu/20260821-080906-imu-motion` t=6.040s) で観測した跳躍。
    // 20.1 ms で yaw だけ -169.03 deg 飛び、roll/pitch はほぼ動かなかった。
    constexpr auto JUMP = -169.03 * boost::math::constants::pi<double>() / 180.0;

    feedback.moment(
        Eigen::Quaterniond::Identity(), heading_quat(0.0), Eigen::Vector3d::Zero(), 0.0, DT, true);
    const auto jumped = feedback.moment(
        Eigen::Quaterniond::Identity(), heading_quat(JUMP), Eigen::Vector3d::Zero(), 0.0, DT, true);

    ASSERT_TRUE(jumped);
    EXPECT_TRUE(feedback.yaw_was_relatched());
    // 追いかけない。追いかけると跳躍がそのまま「180 度回れ」という指令に化ける。
    EXPECT_NEAR(jumped->z(), 0.0, EPS);
    EXPECT_NEAR(feedback.latched_yaw(), JUMP, 1e-9);
}

TEST(AttitudeFeedbackHoldYawTest, NormalTrackingErrorDoesNotRelatch) {
    const auto feedback = AttitudeFeedback{};
    // 実機の追従誤差は最大でも 29.2 deg (docs/known_issues.md B-14 まわり)。既定のクランプ
    // 90 deg では誤爆しないこと。
    constexpr auto ERROR = 29.2 * boost::math::constants::pi<double>() / 180.0;

    feedback.moment(
        Eigen::Quaterniond::Identity(), heading_quat(0.0), Eigen::Vector3d::Zero(), 0.0, DT, true);
    const auto tracking = feedback.moment(
        Eigen::Quaterniond::Identity(), heading_quat(ERROR), Eigen::Vector3d::Zero(), 0.0, DT,
        true);

    ASSERT_TRUE(tracking);
    EXPECT_FALSE(feedback.yaw_was_relatched());
    EXPECT_LT(tracking->z(), 0.0);
    EXPECT_NEAR(feedback.latched_yaw(), 0.0, 1e-9);
}

TEST(AttitudeFeedbackHoldYawTest, HoldErrorWrapsAcrossPi) {
    const auto feedback = AttitudeFeedback{};
    constexpr auto PI = boost::math::constants::pi<double>();

    // +pi のすぐ手前でラッチし、-pi 側へ 0.2 rad だけ回った。折り返しを跨ぐが実際の誤差は
    // 0.2 rad しかないので、ラッチし直さず小さな戻しが出るだけ。
    feedback.moment(
        Eigen::Quaterniond::Identity(), heading_quat(PI - 0.1), Eigen::Vector3d::Zero(), 0.0, DT,
        true);
    const auto wrapped = feedback.moment(
        Eigen::Quaterniond::Identity(), heading_quat(-PI + 0.1), Eigen::Vector3d::Zero(), 0.0, DT,
        true);

    ASSERT_TRUE(wrapped);
    EXPECT_FALSE(feedback.yaw_was_relatched());
    EXPECT_LT(wrapped->z(), 0.0);
    EXPECT_NEAR(std::abs(wrapped->z()), 0.2, 1e-9);
}

TEST(AttitudeFeedbackHoldYawTest, DroppingHoldClearsTheLatch) {
    const auto feedback = AttitudeFeedback{};

    feedback.moment(
        Eigen::Quaterniond::Identity(), heading_quat(0.5), Eigen::Vector3d::Zero(), 0.0, DT, true);
    ASSERT_TRUE(feedback.holding_yaw());

    feedback.moment(
        Eigen::Quaterniond::Identity(), heading_quat(0.5), Eigen::Vector3d::Zero(), 0.0, DT, false);
    EXPECT_FALSE(feedback.holding_yaw());

    // 再度立てたら、そのときの方位を改めてラッチする (古い値を持ち越さない)。
    feedback.moment(
        Eigen::Quaterniond::Identity(), heading_quat(-0.3), Eigen::Vector3d::Zero(), 0.0, DT, true);
    EXPECT_NEAR(feedback.latched_yaw(), -0.3, 1e-9);
}

TEST(AttitudeFeedbackHoldYawTest, ResetDropsTheLatch) {
    const auto feedback = AttitudeFeedback{};

    feedback.moment(
        Eigen::Quaterniond::Identity(), heading_quat(0.5), Eigen::Vector3d::Zero(), 0.0, DT, true);
    ASSERT_TRUE(feedback.holding_yaw());

    // disarm / モード切替で持ち越さない。
    feedback.reset();
    EXPECT_FALSE(feedback.holding_yaw());
}

TEST(AttitudeFeedbackHoldYawTest, HoldDoesNotDisturbRollAndPitch) {
    const auto feedback = AttitudeFeedback{};
    const auto current = heading_quat(0.6);

    const auto rate_only = feedback.moment(
        Eigen::Quaterniond::Identity(), current, Eigen::Vector3d::Zero(), 0.0, DT, false);
    const auto held = feedback.moment(
        Eigen::Quaterniond::Identity(), current, Eigen::Vector3d::Zero(), 0.0, DT, true);

    ASSERT_TRUE(rate_only);
    ASSERT_TRUE(held);
    // yaw の保持は roll/pitch の軸に影響しない。
    EXPECT_NEAR(held->x(), rate_only->x(), EPS);
    EXPECT_NEAR(held->y(), rate_only->y(), EPS);
}

}  // namespace sinsei_umiusi_control::test::controller::logic::attitude

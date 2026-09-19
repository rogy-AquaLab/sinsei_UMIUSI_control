#include "sinsei_umiusi_control/controller/logic/attitude/attitude_feedback.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <limits>

namespace sinsei_umiusi_control::test::controller::logic::attitude {

using sinsei_umiusi_control::controller::logic::attitude::AttitudeFeedback;

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
    const auto moment = feedback.moment(
        target, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0.0);

    ASSERT_TRUE(moment);
    EXPECT_GT(moment->x(), 0.0);
    EXPECT_NEAR(moment->y(), 0.0, EPS);
}

TEST(AttitudeFeedbackTest, PositivePitchTargetProducesPositivePitchMoment) {
    const auto feedback = AttitudeFeedback{};
    const auto target = Eigen::Quaterniond{Eigen::AngleAxisd{ANGLE, Eigen::Vector3d::UnitY()}};
    const auto moment = feedback.moment(
        target, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0.0);

    ASSERT_TRUE(moment);
    EXPECT_NEAR(moment->x(), 0.0, EPS);
    EXPECT_GT(moment->y(), 0.0);
}

TEST(AttitudeFeedbackTest, PositiveCurrentRollProducesRestoringNegativeRollMoment) {
    const auto feedback = AttitudeFeedback{};
    const auto current = Eigen::Quaterniond{Eigen::AngleAxisd{ANGLE, Eigen::Vector3d::UnitX()}};
    const auto moment = feedback.moment(
        Eigen::Quaterniond::Identity(), current, Eigen::Vector3d::Zero(), 0.0);

    ASSERT_TRUE(moment);
    EXPECT_LT(moment->x(), 0.0);
    EXPECT_NEAR(moment->y(), 0.0, EPS);
}

TEST(AttitudeFeedbackTest, PositiveCurrentPitchProducesRestoringNegativePitchMoment) {
    const auto feedback = AttitudeFeedback{};
    const auto current = Eigen::Quaterniond{Eigen::AngleAxisd{ANGLE, Eigen::Vector3d::UnitY()}};
    const auto moment = feedback.moment(
        Eigen::Quaterniond::Identity(), current, Eigen::Vector3d::Zero(), 0.0);

    ASSERT_TRUE(moment);
    EXPECT_NEAR(moment->x(), 0.0, EPS);
    EXPECT_LT(moment->y(), 0.0);
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
    const auto current = Eigen::Quaterniond{
        Eigen::AngleAxisd{1.0, Eigen::Vector3d::UnitZ()}};
    const auto target = Eigen::Quaterniond{
        Eigen::AngleAxisd{-0.7, Eigen::Vector3d::UnitZ()}};
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

    const auto positive = feedback.moment(
        target, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0.0);
    const auto negative = feedback.moment(
        negative_target, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0.0);

    ASSERT_TRUE(positive);
    ASSERT_TRUE(negative);
    EXPECT_TRUE(positive->isApprox(*negative, EPS));
}

TEST(AttitudeFeedbackTest, RejectsInvalidQuaternionAndNonFiniteInput) {
    const auto feedback = AttitudeFeedback{};
    const auto zero_quaternion = Eigen::Quaterniond{0.0, 0.0, 0.0, 0.0};
    const auto nan = std::numeric_limits<double>::quiet_NaN();

    EXPECT_FALSE(feedback.moment(
        zero_quaternion, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), 0.0));
    EXPECT_FALSE(feedback.moment(
        Eigen::Quaterniond::Identity(), Eigen::Quaterniond::Identity(),
        Eigen::Vector3d{nan, 0.0, 0.0}, 0.0));
}

}  // namespace sinsei_umiusi_control::test::controller::logic::attitude

#include "sinsei_umiusi_control/controller/logic/thruster/servo_angle_estimator.hpp"

#include <gtest/gtest.h>

#include <boost/math/constants/constants.hpp>
#include <limits>
#include <stdexcept>

namespace sinsei_umiusi_control::test::controller::logic::thruster {

constexpr auto HALF_PI = boost::math::constants::pi<double>() / 2.0;
constexpr auto MIN_ANGLE = -HALF_PI;
constexpr auto MAX_ANGLE = HALF_PI;
using CommandedAngle = state::thruster::servo::CommandedAngle;
using ServoAngleEstimator = sinsei_umiusi_control::controller::logic::thruster::ServoAngleEstimator;

TEST(ServoAngleEstimatorTest, StartsWithoutAnEstimate) {
    auto estimator = ServoAngleEstimator{1.0};

    EXPECT_FALSE(estimator.update(CommandedAngle{0.0}, 0.0));
}

TEST(ServoAngleEstimatorTest, BecomesAvailableAfterAllPossibleAnglesReachCommand) {
    auto estimator = ServoAngleEstimator{1.0};

    EXPECT_FALSE(estimator.update(CommandedAngle{0.0}, MAX_ANGLE - 0.01));
    const auto estimate = estimator.update(CommandedAngle{0.0}, 0.02);

    ASSERT_TRUE(estimate);
    EXPECT_DOUBLE_EQ(estimate->value, 0.0);
}

TEST(ServoAngleEstimatorTest, UsesFullRangeForAnEndStopCommand) {
    auto estimator = ServoAngleEstimator{2.0};
    const auto full_range_duration = (MAX_ANGLE - MIN_ANGLE) / 2.0;

    EXPECT_FALSE(estimator.update(CommandedAngle{MAX_ANGLE}, full_range_duration - 0.01));
    const auto estimate = estimator.update(CommandedAngle{MAX_ANGLE}, 0.02);

    ASSERT_TRUE(estimate);
    EXPECT_DOUBLE_EQ(estimate->value, MAX_ANGLE);
}

TEST(ServoAngleEstimatorTest, TracksCommandsAfterInitialization) {
    auto estimator = ServoAngleEstimator{1.0};
    ASSERT_TRUE(estimator.update(CommandedAngle{0.0}, MAX_ANGLE));

    const auto estimate = estimator.update(CommandedAngle{1.0}, 0.25);

    ASSERT_TRUE(estimate);
    EXPECT_DOUBLE_EQ(estimate->value, 0.25);
}

TEST(ServoAngleEstimatorTest, TracksPossibleRangeWhenCommandChangesBeforeInitialization) {
    auto estimator = ServoAngleEstimator{1.0};

    EXPECT_FALSE(estimator.update(CommandedAngle{MAX_ANGLE}, 1.0));
    EXPECT_FALSE(estimator.update(CommandedAngle{MIN_ANGLE}, 1.0));
    const auto estimate = estimator.update(CommandedAngle{MIN_ANGLE}, 2.2);

    ASSERT_TRUE(estimate);
    EXPECT_DOUBLE_EQ(estimate->value, MIN_ANGLE);
}

TEST(ServoAngleEstimatorTest, ResetMakesAngleUnknownAgain) {
    auto estimator = ServoAngleEstimator{1.0};
    ASSERT_TRUE(estimator.update(CommandedAngle{0.0}, MAX_ANGLE));

    estimator.reset();

    EXPECT_FALSE(estimator.update(CommandedAngle{0.0}, 0.0));
}

TEST(ServoAngleEstimatorTest, ZeroVelocityKeepsAngleUnknown) {
    auto estimator = ServoAngleEstimator{0.0};

    EXPECT_FALSE(estimator.update(CommandedAngle{0.0}, 100.0));
}

TEST(ServoAngleEstimatorTest, InvalidInputResetsTheEstimate) {
    auto estimator = ServoAngleEstimator{1.0};
    ASSERT_TRUE(estimator.update(CommandedAngle{0.0}, MAX_ANGLE));

    EXPECT_FALSE(estimator.update(CommandedAngle{std::numeric_limits<double>::quiet_NaN()}, 0.1));
    EXPECT_FALSE(estimator.update(CommandedAngle{0.0}, 0.0));
}

TEST(ServoAngleEstimatorTest, OutOfRangeCommandResetsTheEstimate) {
    auto estimator = ServoAngleEstimator{1.0};
    ASSERT_TRUE(estimator.update(CommandedAngle{0.0}, MAX_ANGLE));

    EXPECT_FALSE(estimator.update(CommandedAngle{MAX_ANGLE + 0.01}, 0.1));
    EXPECT_FALSE(estimator.update(CommandedAngle{0.0}, 0.0));
}

TEST(ServoAngleEstimatorTest, RejectsInvalidVelocity) {
    EXPECT_THROW(ServoAngleEstimator{-1.0}, std::invalid_argument);
    EXPECT_THROW(
        ServoAngleEstimator{std::numeric_limits<double>::infinity()}, std::invalid_argument);
}

}  // namespace sinsei_umiusi_control::test::controller::logic::thruster

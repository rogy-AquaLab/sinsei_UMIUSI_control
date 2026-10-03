#include "sinsei_umiusi_control/controller/logic/attitude/mixer.hpp"

#include <gtest/gtest.h>

#include <array>
#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <limits>
#include <optional>

namespace sinsei_umiusi_control::test::controller::logic::attitude {

using sinsei_umiusi_control::controller::AttitudeController;
using sinsei_umiusi_control::controller::logic::attitude::mix_to_thrusters;
using sinsei_umiusi_control::controller::logic::attitude::MixerParameters;
using sinsei_umiusi_control::controller::logic::attitude::MixerState;
using EstimatedAngle = state::thruster::servo::EstimatedAngle;
using MaxAngularVelocity = state::thruster::servo::MaxAngularVelocity;

constexpr auto HALF_PI = boost::math::constants::pi<double>() / 2.0;
constexpr auto ROOT_TWO = boost::math::constants::root_two<double>();
constexpr auto EPS = 1e-12;

auto parameters(
    double direction_deadband_deg = 5.0, double reversal_deadband_deg = 10.0,
    double retarget_thrust_enter = 0.10,
    double retarget_thrust_exit = 0.06) -> MixerParameters {
    constexpr auto DEG_TO_RAD = boost::math::constants::pi<double>() / 180.0;
    return {
        direction_deadband_deg * DEG_TO_RAD,
        reversal_deadband_deg * DEG_TO_RAD,
        retarget_thrust_enter,
        retarget_thrust_exit,
    };
}

auto estimates(double angle) -> std::array<std::optional<EstimatedAngle>, 4> {
    return {
        EstimatedAngle{angle},
        EstimatedAngle{angle},
        EstimatedAngle{angle},
        EstimatedAngle{angle},
    };
}

auto velocities(double velocity) -> std::array<MaxAngularVelocity, 4> {
    return {
        MaxAngularVelocity{velocity},
        MaxAngularVelocity{velocity},
        MaxAngularVelocity{velocity},
        MaxAngularVelocity{velocity},
    };
}

auto mix(
    const Eigen::Vector<double, 6> & request,
    const std::array<std::optional<EstimatedAngle>, 4> & servo_estimates,
    const std::array<MaxAngularVelocity, 4> & servo_velocities, double duration,
    const MixerParameters & mixer_parameters = parameters()) -> AttitudeController::Output {
    auto mixer_state = MixerState{};
    return mix_to_thrusters(
        request, servo_estimates, servo_velocities, duration, mixer_parameters, mixer_state);
}

auto servo_direction_request(double normalized_thrust, double angle)
    -> Eigen::Vector<double, 6> {
    const auto thrust = normalized_thrust * ROOT_TWO;
    return {
        thrust * std::sin(angle),
        0.0,
        thrust * std::cos(angle),
        0.0,
        0.0,
        0.0,
    };
}

TEST(AttitudeMixerTest, HoldsZeroThrustAndHomesServosUntilEveryEstimateIsAvailable) {
    auto servo_estimates = estimates(0.4);
    servo_estimates[2] = std::nullopt;
    const Eigen::Vector<double, 6> request{1.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    const auto output = mix(request, servo_estimates, velocities(1.57), 0.02);

    for (size_t i = 0; i < 4; ++i) {
        EXPECT_DOUBLE_EQ(output.cmd.esc_thrusts[i].value, 0.0);
        EXPECT_DOUBLE_EQ(output.cmd.servo_angles[i].value, 0.0);
    }
}

TEST(AttitudeMixerTest, LimitsServoCommandByMeasuredVelocityAndDuration) {
    const Eigen::Vector<double, 6> request{1.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    const auto output = mix(request, estimates(0.0), velocities(1.0), 0.1);

    EXPECT_NEAR(output.cmd.servo_angles[0].value, 0.1, EPS);
    EXPECT_NEAR(output.cmd.servo_angles[1].value, 0.1, EPS);
    EXPECT_NEAR(output.cmd.servo_angles[2].value, -0.1, EPS);
    EXPECT_NEAR(output.cmd.servo_angles[3].value, -0.1, EPS);
    for (const auto & thrust : output.cmd.esc_thrusts) {
        EXPECT_NEAR(thrust.value, 0.0, EPS);
    }
}

TEST(AttitudeMixerTest, ProjectsRequestedForceOntoCurrentServoAxis) {
    const Eigen::Vector<double, 6> request{0.0, 0.0, 0.0, 0.5, 0.0, 0.0};

    const auto output = mix(request, estimates(0.0), velocities(1.0), 0.02);

    EXPECT_NEAR(output.cmd.esc_thrusts[0].value, -0.5, EPS);
    EXPECT_NEAR(output.cmd.esc_thrusts[1].value, -0.5, EPS);
    EXPECT_NEAR(output.cmd.esc_thrusts[2].value, 0.5, EPS);
    EXPECT_NEAR(output.cmd.esc_thrusts[3].value, 0.5, EPS);
}

TEST(AttitudeMixerTest, ScalesVerticalTranslationLikeHorizontalTranslation) {
    const Eigen::Vector<double, 6> request{0.0, 0.0, 0.0, 0.0, 0.0, 0.5};

    const auto output = mix(request, estimates(HALF_PI), velocities(1.0), 0.02);

    for (const auto & thrust : output.cmd.esc_thrusts) {
        EXPECT_NEAR(thrust.value, 0.5, EPS);
    }
}

TEST(AttitudeMixerTest, BuildsVerticalThrustAsServosActuallyMove) {
    auto servo_estimates = estimates(0.1);
    servo_estimates[2] = EstimatedAngle{-0.1};
    servo_estimates[3] = EstimatedAngle{-0.1};
    const Eigen::Vector<double, 6> request{1.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    const auto output = mix(request, servo_estimates, velocities(1.0), 0.1);
    const auto expected_thrust = std::sin(0.1) / ROOT_TWO;

    for (const auto & thrust : output.cmd.esc_thrusts) {
        EXPECT_NEAR(thrust.value, expected_thrust, EPS);
    }
}

auto limited_parameters(double esc_thrust_limit) -> MixerParameters {
    auto limited = parameters();
    limited.esc_thrust_limit = esc_thrust_limit;
    return limited;
}

TEST(AttitudeMixerTest, ShrinksTranslationToKeepYawMomentUnderThrustLimit) {
    // 前進 1.0 + yaw。上限 0.5 で並進だけを縮め、全基が同じ値に張り付かないこと。
    const Eigen::Vector<double, 6> request{0.0, 0.0, 0.2, 1.0, 0.0, 0.0};

    const auto output =
        mix(request, estimates(0.0), velocities(1.0), 0.02, limited_parameters(0.5));

    auto yaw_sum = 0.0;
    for (const auto & thrust : output.cmd.esc_thrusts) {
        EXPECT_LE(std::abs(thrust.value), 0.5 + EPS);
        yaw_sum += thrust.value;
    }
    EXPECT_NEAR(yaw_sum, 4.0 * 0.2 / ROOT_TWO, EPS);
    EXPECT_NEAR(output.cmd.esc_thrusts[2].value, 0.5, EPS);
    EXPECT_NEAR(output.cmd.esc_thrusts[3].value, 0.5, EPS);
    EXPECT_LT(output.cmd.esc_thrusts[0].value, 0.0);
    EXPECT_GT(output.cmd.esc_thrusts[0].value, -0.5);
}

TEST(AttitudeMixerTest, ScalesMomentUniformlyWhenMomentAloneExceedsThrustLimit) {
    const Eigen::Vector<double, 6> request{0.0, 0.0, 1.0, 1.0, 0.0, 0.0};

    const auto output =
        mix(request, estimates(0.0), velocities(1.0), 0.02, limited_parameters(0.5));

    for (const auto & thrust : output.cmd.esc_thrusts) {
        EXPECT_NEAR(thrust.value, 0.5, EPS);
    }
}

TEST(AttitudeMixerTest, LeavesRequestsWithinThrustLimitUnchanged) {
    const Eigen::Vector<double, 6> request{0.0, 0.0, 0.1, 0.3, 0.0, 0.0};

    const auto limited =
        mix(request, estimates(0.0), velocities(1.0), 0.02, limited_parameters(0.5));
    const auto unlimited = mix(request, estimates(0.0), velocities(1.0), 0.02);

    for (size_t i = 0; i < 4; ++i) {
        EXPECT_NEAR(limited.cmd.esc_thrusts[i].value, unlimited.cmd.esc_thrusts[i].value, EPS);
    }
}

TEST(AttitudeMixerTest, InvalidThrustLimitStopsThrust) {
    const Eigen::Vector<double, 6> request{0.0, 0.0, 0.0, 1.0, 0.0, 0.0};

    const auto output =
        mix(request, estimates(0.4), velocities(1.0), 0.02, limited_parameters(0.0));

    for (size_t i = 0; i < 4; ++i) {
        EXPECT_DOUBLE_EQ(output.cmd.esc_thrusts[i].value, 0.0);
        EXPECT_DOUBLE_EQ(output.cmd.servo_angles[i].value, 0.4);
    }
}

TEST(AttitudeMixerTest, DoesNotCrossTheFullServoRangeForEndStopNoise) {
    const Eigen::Vector<double, 6> request{0.0, 0.0, -0.01, 0.0, 0.0, 1.0 / ROOT_TWO};

    const auto output = mix(request, estimates(HALF_PI), velocities(1.57), 0.02);

    for (size_t i = 0; i < 4; ++i) {
        EXPECT_NEAR(output.cmd.servo_angles[i].value, HALF_PI, EPS);
        EXPECT_NEAR(output.cmd.esc_thrusts[i].value, 1.0 / ROOT_TWO, EPS);
    }
}

TEST(AttitudeMixerTest, IgnoresSmallDirectionChangesAtAnyServoAngle) {
    constexpr auto FOUR_DEGREES = boost::math::constants::pi<double>() / 45.0;
    const Eigen::Vector<double, 6> request{
        0.0, 0.0, std::cos(FOUR_DEGREES), 0.0, 0.0, std::sin(FOUR_DEGREES) / ROOT_TWO};

    const auto output = mix(request, estimates(0.0), velocities(1.57), 0.02);

    for (const auto & angle : output.cmd.servo_angles) {
        EXPECT_DOUBLE_EQ(angle.value, 0.0);
    }
}

TEST(AttitudeMixerTest, MovesServoOnceDirectionChangeExceedsDeadband) {
    constexpr auto SIX_DEGREES = boost::math::constants::pi<double>() / 30.0;
    const Eigen::Vector<double, 6> request{
        0.0, 0.0, std::cos(SIX_DEGREES), 0.0, 0.0, std::sin(SIX_DEGREES) / ROOT_TWO};

    const auto output = mix(request, estimates(0.0), velocities(1.57), 0.1);

    for (const auto & angle : output.cmd.servo_angles) {
        EXPECT_NEAR(angle.value, SIX_DEGREES, EPS);
    }
}

TEST(AttitudeMixerTest, UsesConfiguredDirectionDeadband) {
    constexpr auto SIX_DEGREES = boost::math::constants::pi<double>() / 30.0;
    const Eigen::Vector<double, 6> request{
        0.0, 0.0, std::cos(SIX_DEGREES), 0.0, 0.0, std::sin(SIX_DEGREES) / ROOT_TWO};

    const auto output = mix(request, estimates(0.0), velocities(1.57), 0.1, parameters(7.0));

    for (const auto & angle : output.cmd.servo_angles) {
        EXPECT_DOUBLE_EQ(angle.value, 0.0);
    }
}

TEST(AttitudeMixerTest, HoldsServoDirectionBelowRetargetEnterThreshold) {
    constexpr auto TARGET_ANGLE = boost::math::constants::pi<double>() / 6.0;
    const auto request = servo_direction_request(0.09, TARGET_ANGLE);
    auto mixer_state = MixerState{};

    const auto output = mix_to_thrusters(
        request, estimates(0.0), velocities(1.57), 0.1, parameters(), mixer_state);

    for (size_t i = 0; i < 4; ++i) {
        EXPECT_DOUBLE_EQ(output.cmd.servo_angles[i].value, 0.0);
        EXPECT_FALSE(mixer_state.servo_retargeting[i]);
        EXPECT_NE(output.cmd.esc_thrusts[i].value, 0.0);
    }
}

TEST(AttitudeMixerTest, RetargetThrustThresholdsProvideHysteresis) {
    constexpr auto TARGET_ANGLE = boost::math::constants::pi<double>() / 6.0;
    auto mixer_state = MixerState{};

    const auto above_enter = mix_to_thrusters(
        servo_direction_request(0.11, TARGET_ANGLE), estimates(0.0), velocities(1.57), 0.1,
        parameters(), mixer_state);
    for (size_t i = 0; i < 4; ++i) {
        EXPECT_TRUE(mixer_state.servo_retargeting[i]);
        EXPECT_NE(above_enter.cmd.servo_angles[i].value, 0.0);
    }

    const auto between_thresholds = mix_to_thrusters(
        servo_direction_request(0.08, TARGET_ANGLE), estimates(0.0), velocities(1.57), 0.1,
        parameters(), mixer_state);
    for (size_t i = 0; i < 4; ++i) {
        EXPECT_TRUE(mixer_state.servo_retargeting[i]);
        EXPECT_NE(between_thresholds.cmd.servo_angles[i].value, 0.0);
    }

    const auto below_exit = mix_to_thrusters(
        servo_direction_request(0.05, TARGET_ANGLE), estimates(0.0), velocities(1.57), 0.1,
        parameters(), mixer_state);
    for (size_t i = 0; i < 4; ++i) {
        EXPECT_FALSE(mixer_state.servo_retargeting[i]);
        EXPECT_DOUBLE_EQ(below_exit.cmd.servo_angles[i].value, 0.0);
    }
}

TEST(AttitudeMixerTest, TraversesServoRangeWhenEndStopApproximationIsInsufficient) {
    const Eigen::Vector<double, 6> request{0.0, 0.0, -0.2, 0.0, 0.0, 1.0 / ROOT_TWO};

    const auto output = mix(request, estimates(HALF_PI), velocities(1.0), 0.1);

    for (const auto & angle : output.cmd.servo_angles) {
        EXPECT_NEAR(angle.value, HALF_PI - 0.1, EPS);
    }
}

TEST(AttitudeMixerTest, UsesConfiguredReversalDeadband) {
    const Eigen::Vector<double, 6> request{0.0, 0.0, -0.2, 0.0, 0.0, 1.0 / ROOT_TWO};

    const auto output =
        mix(request, estimates(HALF_PI), velocities(1.0), 0.1, parameters(5.0, 15.0));

    for (const auto & angle : output.cmd.servo_angles) {
        EXPECT_NEAR(angle.value, HALF_PI, EPS);
    }
}

TEST(AttitudeMixerTest, RejectsInvalidTimingOrServoState) {
    const Eigen::Vector<double, 6> request{0.0, 0.0, 0.0, 1.0, 0.0, 0.0};
    auto invalid_estimates = estimates(0.0);
    invalid_estimates[0] = EstimatedAngle{HALF_PI + 0.01};

    const auto invalid_time = mix(request, estimates(0.0), velocities(1.0), -0.1);
    const auto invalid_angle = mix(request, invalid_estimates, velocities(1.0), 0.02);

    for (size_t i = 0; i < 4; ++i) {
        EXPECT_DOUBLE_EQ(invalid_time.cmd.esc_thrusts[i].value, 0.0);
        EXPECT_DOUBLE_EQ(invalid_angle.cmd.esc_thrusts[i].value, 0.0);
    }
}

TEST(AttitudeMixerTest, InvalidInputStopsThrustWithoutMovingKnownServos) {
    Eigen::Vector<double, 6> request{0.0, 0.0, 0.0, 1.0, 0.0, 0.0};
    request[0] = std::numeric_limits<double>::quiet_NaN();

    const auto output = mix(request, estimates(0.4), velocities(1.0), 0.02);

    for (size_t i = 0; i < 4; ++i) {
        EXPECT_DOUBLE_EQ(output.cmd.esc_thrusts[i].value, 0.0);
        EXPECT_DOUBLE_EQ(output.cmd.servo_angles[i].value, 0.4);
    }
}

TEST(AttitudeMixerTest, InvalidParametersStopThrustWithoutMovingKnownServos) {
    const Eigen::Vector<double, 6> request{0.0, 0.0, 0.0, 1.0, 0.0, 0.0};
    auto invalid_parameters = parameters();
    invalid_parameters.servo_direction_deadband = std::numeric_limits<double>::quiet_NaN();

    const auto output = mix(request, estimates(0.4), velocities(1.0), 0.02, invalid_parameters);

    for (size_t i = 0; i < 4; ++i) {
        EXPECT_DOUBLE_EQ(output.cmd.esc_thrusts[i].value, 0.0);
        EXPECT_DOUBLE_EQ(output.cmd.servo_angles[i].value, 0.4);
    }
}

}  // namespace sinsei_umiusi_control::test::controller::logic::attitude

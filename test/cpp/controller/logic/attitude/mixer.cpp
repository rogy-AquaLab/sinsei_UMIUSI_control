#include "sinsei_umiusi_control/controller/logic/attitude/mixer.hpp"

#include <gtest/gtest.h>

#include <array>
#include <boost/math/constants/constants.hpp>
#include <limits>
#include <optional>

namespace sinsei_umiusi_control::test::controller::logic::attitude {

using sinsei_umiusi_control::controller::logic::attitude::mix_to_thrusters;
using EstimatedAngle = state::thruster::servo::EstimatedAngle;
using MaxAngularVelocity = state::thruster::servo::MaxAngularVelocity;

constexpr auto HALF_PI = boost::math::constants::pi<double>() / 2.0;
constexpr auto ROOT_TWO = boost::math::constants::root_two<double>();
constexpr auto EPS = 1e-12;

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

TEST(AttitudeMixerTest, HoldsZeroThrustAndHomesServosUntilEveryEstimateIsAvailable) {
    auto servo_estimates = estimates(0.4);
    servo_estimates[2] = std::nullopt;
    const Eigen::Vector<double, 6> request{1.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    const auto output = mix_to_thrusters(request, servo_estimates, velocities(1.57), 0.02);

    for (size_t i = 0; i < 4; ++i) {
        EXPECT_DOUBLE_EQ(output.cmd.esc_thrusts[i].value, 0.0);
        EXPECT_DOUBLE_EQ(output.cmd.servo_angles[i].value, 0.0);
    }
}

TEST(AttitudeMixerTest, LimitsServoCommandByMeasuredVelocityAndDuration) {
    const Eigen::Vector<double, 6> request{1.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    const auto output = mix_to_thrusters(request, estimates(0.0), velocities(1.0), 0.1);

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

    const auto output = mix_to_thrusters(request, estimates(0.0), velocities(1.0), 0.02);

    EXPECT_NEAR(output.cmd.esc_thrusts[0].value, -0.5, EPS);
    EXPECT_NEAR(output.cmd.esc_thrusts[1].value, -0.5, EPS);
    EXPECT_NEAR(output.cmd.esc_thrusts[2].value, 0.5, EPS);
    EXPECT_NEAR(output.cmd.esc_thrusts[3].value, 0.5, EPS);
}

TEST(AttitudeMixerTest, BuildsVerticalThrustAsServosActuallyMove) {
    auto servo_estimates = estimates(0.1);
    servo_estimates[2] = EstimatedAngle{-0.1};
    servo_estimates[3] = EstimatedAngle{-0.1};
    const Eigen::Vector<double, 6> request{1.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    const auto output = mix_to_thrusters(request, servo_estimates, velocities(1.0), 0.1);
    const auto expected_thrust = std::sin(0.1) / ROOT_TWO;

    for (const auto & thrust : output.cmd.esc_thrusts) {
        EXPECT_NEAR(thrust.value, expected_thrust, EPS);
    }
}

TEST(AttitudeMixerTest, DoesNotCrossTheFullServoRangeForEndStopNoise) {
    const Eigen::Vector<double, 6> request{0.0, 0.0, -0.01, 0.0, 0.0, 1.0};

    const auto output = mix_to_thrusters(request, estimates(HALF_PI), velocities(1.57), 0.02);

    for (size_t i = 0; i < 4; ++i) {
        EXPECT_NEAR(output.cmd.servo_angles[i].value, HALF_PI, EPS);
        EXPECT_NEAR(output.cmd.esc_thrusts[i].value, 1.0 / ROOT_TWO, EPS);
    }
}

TEST(AttitudeMixerTest, IgnoresSmallDirectionChangesAtAnyServoAngle) {
    const Eigen::Vector<double, 6> request{0.0, 0.0, std::cos(0.01), 0.0, 0.0, std::sin(0.01)};

    const auto output = mix_to_thrusters(request, estimates(0.0), velocities(1.57), 0.02);

    for (const auto & angle : output.cmd.servo_angles) {
        EXPECT_DOUBLE_EQ(angle.value, 0.0);
    }
}

TEST(AttitudeMixerTest, TraversesServoRangeWhenEndStopApproximationIsInsufficient) {
    const Eigen::Vector<double, 6> request{0.0, 0.0, -0.2, 0.0, 0.0, 1.0};

    const auto output = mix_to_thrusters(request, estimates(HALF_PI), velocities(1.0), 0.1);

    for (const auto & angle : output.cmd.servo_angles) {
        EXPECT_NEAR(angle.value, HALF_PI - 0.1, EPS);
    }
}

TEST(AttitudeMixerTest, RejectsInvalidTimingOrServoState) {
    const Eigen::Vector<double, 6> request{0.0, 0.0, 0.0, 1.0, 0.0, 0.0};
    auto invalid_estimates = estimates(0.0);
    invalid_estimates[0] = EstimatedAngle{HALF_PI + 0.01};

    const auto invalid_time = mix_to_thrusters(request, estimates(0.0), velocities(1.0), -0.1);
    const auto invalid_angle = mix_to_thrusters(request, invalid_estimates, velocities(1.0), 0.02);

    for (size_t i = 0; i < 4; ++i) {
        EXPECT_DOUBLE_EQ(invalid_time.cmd.esc_thrusts[i].value, 0.0);
        EXPECT_DOUBLE_EQ(invalid_angle.cmd.esc_thrusts[i].value, 0.0);
    }
}

TEST(AttitudeMixerTest, InvalidInputStopsThrustWithoutMovingKnownServos) {
    Eigen::Vector<double, 6> request{0.0, 0.0, 0.0, 1.0, 0.0, 0.0};
    request[0] = std::numeric_limits<double>::quiet_NaN();

    const auto output = mix_to_thrusters(request, estimates(0.4), velocities(1.0), 0.02);

    for (size_t i = 0; i < 4; ++i) {
        EXPECT_DOUBLE_EQ(output.cmd.esc_thrusts[i].value, 0.0);
        EXPECT_DOUBLE_EQ(output.cmd.servo_angles[i].value, 0.4);
    }
}

}  // namespace sinsei_umiusi_control::test::controller::logic::attitude

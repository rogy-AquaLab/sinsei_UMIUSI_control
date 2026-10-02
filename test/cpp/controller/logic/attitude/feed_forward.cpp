#include "sinsei_umiusi_control/controller/logic/attitude/feed_forward.hpp"

#include <gtest/gtest.h>

#include <boost/math/constants/constants.hpp>
#include <cmath>

namespace sinsei_umiusi_control::test::controller::logic::attitude {

using sinsei_umiusi_control::controller::AttitudeController;
using sinsei_umiusi_control::controller::logic::attitude::FeedForward;
using sinsei_umiusi_control::controller::logic::attitude::FeedForwardGains;
using sinsei_umiusi_control::controller::logic::attitude::MixerParameters;
using EstimatedAngle = state::thruster::servo::EstimatedAngle;
using MaxAngularVelocity = state::thruster::servo::MaxAngularVelocity;

constexpr auto HALF_PI = boost::math::constants::pi<double>() / 2.0;
constexpr auto ROOT_TWO = boost::math::constants::root_two<double>();
constexpr auto EPS = 1e-12;

auto mixer_parameters() -> MixerParameters {
    constexpr auto DEG_TO_RAD = boost::math::constants::pi<double>() / 180.0;
    return {5.0 * DEG_TO_RAD, 10.0 * DEG_TO_RAD, 0.10, 0.06};
}

auto make_input() -> AttitudeController::Input {
    auto input = AttitudeController::Input{};
    input.cmd.target_attitude.w = 1.0;
    input.state.servo_estimated_angles.fill(EstimatedAngle{0.0});
    input.state.servo_max_angular_velocities.fill(MaxAngularVelocity{1.57});
    return input;
}

TEST(FeedForwardTest, RestoresUiRollCommandScaleFromQuaternionHalfAngle) {
    constexpr auto ROLL = 0.3;
    auto input = make_input();
    input.cmd.target_attitude.x = std::sin(ROLL / 2.0);
    input.cmd.target_attitude.w = std::cos(ROLL / 2.0);
    input.state.servo_estimated_angles = {
        EstimatedAngle{HALF_PI},
        EstimatedAngle{HALF_PI},
        EstimatedAngle{-HALF_PI},
        EstimatedAngle{-HALF_PI},
    };

    const auto output = FeedForward{FeedForwardGains{}, mixer_parameters()}.update(0.0, 0.02, input);
    const auto expected_thrust = 2.0 * std::sin(ROLL / 2.0) / ROOT_TWO;

    for (const auto & thrust : output.cmd.esc_thrusts) {
        EXPECT_NEAR(thrust.value, expected_thrust, EPS);
    }
}

TEST(FeedForwardTest, PreservesPreviousUiYawCommandRange) {
    auto input = make_input();
    input.cmd.target_attitude.yaw_rate = 1.0;

    const auto output = FeedForward{FeedForwardGains{}, mixer_parameters()}.update(0.0, 0.02, input);
    const auto expected_thrust = 0.2 / ROOT_TWO;

    for (const auto & thrust : output.cmd.esc_thrusts) {
        EXPECT_NEAR(thrust.value, expected_thrust, EPS);
    }
}

TEST(FeedForwardTest, UsesConfiguredYawRateGain) {
    auto input = make_input();
    input.cmd.target_attitude.yaw_rate = 1.0;
    auto gains = FeedForwardGains{};
    gains.k_yaw_rate = 0.1;

    const auto output = FeedForward{gains, mixer_parameters()}.update(0.0, 0.02, input);
    const auto expected_thrust = 0.1 / ROOT_TWO;

    for (const auto & thrust : output.cmd.esc_thrusts) {
        EXPECT_NEAR(thrust.value, expected_thrust, EPS);
    }
}

}  // namespace sinsei_umiusi_control::test::controller::logic::attitude

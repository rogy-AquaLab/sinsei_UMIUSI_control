#include "sinsei_umiusi_control/controller/logic/attitude/feed_forward.hpp"

#include <gtest/gtest.h>

#include <boost/math/constants/constants.hpp>
#include <cmath>

namespace sinsei_umiusi_control::test::controller::logic::attitude {

using sinsei_umiusi_control::controller::AttitudeController;
using sinsei_umiusi_control::controller::logic::attitude::FeedForward;
constexpr auto ROOT_TWO = boost::math::constants::root_two<double>();
constexpr auto EPS = 1e-12;

auto make_input() -> AttitudeController::Input {
    auto input = AttitudeController::Input{};
    input.cmd.target_attitude.w = 1.0;
    return input;
}

TEST(FeedForwardTest, RestoresUiRollCommandScaleFromQuaternionHalfAngle) {
    constexpr auto ROLL = 0.3;
    auto input = make_input();
    input.cmd.target_attitude.x = std::sin(ROLL / 2.0);
    input.cmd.target_attitude.w = std::cos(ROLL / 2.0);
    const auto output = FeedForward{}.update(0.0, 0.02, input);
    const auto expected_thrust = 2.0 * std::sin(ROLL / 2.0) / ROOT_TWO;

    for (const auto & thrust : output.cmd.esc_thrusts) {
        EXPECT_NEAR(thrust.value, expected_thrust, EPS);
    }
}

TEST(FeedForwardTest, PreservesPreviousUiYawCommandRange) {
    auto input = make_input();
    input.cmd.target_attitude.yaw_rate = 1.0;

    const auto output = FeedForward{}.update(0.0, 0.02, input);
    const auto expected_thrust = 0.2 / ROOT_TWO;

    for (const auto & thrust : output.cmd.esc_thrusts) {
        EXPECT_NEAR(thrust.value, expected_thrust, EPS);
    }
}

TEST(FeedForwardTest, MapsUiDpadLeftToLateralThrustPattern) {
    auto input = make_input();
    input.cmd.target_velocity.y = 0.5;

    const auto output = FeedForward{}.update(0.0, 0.02, input);

    EXPECT_NEAR(output.cmd.esc_thrusts[0].value, 0.5, EPS);
    EXPECT_NEAR(output.cmd.esc_thrusts[1].value, -0.5, EPS);
    EXPECT_NEAR(output.cmd.esc_thrusts[2].value, -0.5, EPS);
    EXPECT_NEAR(output.cmd.esc_thrusts[3].value, 0.5, EPS);
    for (const auto & angle : output.cmd.servo_angles) {
        EXPECT_NEAR(angle.value, 0.0, EPS);
    }
}

}  // namespace sinsei_umiusi_control::test::controller::logic::attitude

#include "sinsei_umiusi_control/controller/logic/attitude/feed_forward.hpp"

#include <gtest/gtest.h>

#include <array>
#include <boost/math/constants/constants.hpp>
#include <cmath>

namespace sinsei_umiusi_control::test::controller::logic::attitude {

namespace {

using FeedForward = sinsei_umiusi_control::controller::logic::attitude::FeedForward;
using Input = sinsei_umiusi_control::controller::AttitudeController::Input;
using Output = sinsei_umiusi_control::controller::AttitudeController::Output;

auto decode_components(const Output & output) -> std::array<double, 8> {
    constexpr auto pi = boost::math::constants::pi<double>();
    auto components = std::array<double, 8>{};
    for (auto i = size_t{0}; i < 4; ++i) {
        const auto angle = output.cmd.servo_angles[i].value * pi / 180.0;
        const auto thrust = output.cmd.esc_thrusts[i].value;
        components[2 * i] = thrust * std::cos(angle);
        components[2 * i + 1] = thrust * std::sin(angle);
    }
    return components;
}

auto command_from_components(const std::array<double, 8> & component) -> std::array<double, 6> {
    const auto sqrt_two = boost::math::constants::root_two<double>();
    const auto & [h_lf, v_lf, h_lb, v_lb, h_rb, v_rb, h_rf, v_rf] = component;
    return {
        (v_lf + v_lb - v_rb - v_rf) / 4.0,
        (-v_lf + v_lb + v_rb - v_rf) / 4.0,
        (h_lf + h_lb + h_rb + h_rf) / 4.0,
        sqrt_two * (-h_lf - h_lb + h_rb + h_rf) / 8.0,
        sqrt_two * (h_lf - h_lb - h_rb + h_rf) / 8.0,
        (v_lf + v_lb + v_rb + v_rf) / 4.0,
    };
}

}  // namespace

TEST(FeedForwardTest, HealthyAllocationIsUnchanged) {
    auto input = Input{};
    input.cmd.target_velocity.x = 0.25;

    const auto output = FeedForward{}.update(0.0, 0.0, input);

    EXPECT_DOUBLE_EQ(output.cmd.esc_thrusts[0].value, -0.25);
    EXPECT_DOUBLE_EQ(output.cmd.esc_thrusts[1].value, -0.25);
    EXPECT_DOUBLE_EQ(output.cmd.esc_thrusts[2].value, 0.25);
    EXPECT_DOUBLE_EQ(output.cmd.esc_thrusts[3].value, 0.25);
}

TEST(FeedForwardTest, ReallocatesAllDegreesOfFreedomWithAnySingleDisabledThruster) {
    auto input = Input{};
    input.cmd.target_orientation = {0.05, -0.04, 0.03};
    input.cmd.target_velocity = {-0.02, 0.01, -0.06};

    const auto normalization = boost::math::constants::root_two<double>();
    const auto expected_command = std::array<double, 6>{
        input.cmd.target_orientation.x, input.cmd.target_orientation.y,
        input.cmd.target_orientation.z, input.cmd.target_velocity.x,
        input.cmd.target_velocity.y,    input.cmd.target_velocity.z,
    };

    for (auto disabled_thruster = size_t{0}; disabled_thruster < 4; ++disabled_thruster) {
        SCOPED_TRACE(disabled_thruster);
        const auto output = FeedForward{disabled_thruster}.update(0.0, 0.0, input);
        const auto component = decode_components(output);
        const auto reconstructed_command = command_from_components(component);

        EXPECT_NEAR(output.cmd.esc_thrusts[disabled_thruster].value, 0.0, 1e-12);
        EXPECT_NEAR(output.cmd.servo_angles[disabled_thruster].value, 0.0, 1e-12);
        for (auto i = size_t{0}; i < reconstructed_command.size(); ++i) {
            EXPECT_NEAR(reconstructed_command[i], expected_command[i] / normalization, 1e-12);
        }
    }
}

TEST(FeedForwardTest, ScalesReallocatedOutputsUniformlyToAvoidClipping) {
    auto input = Input{};
    input.cmd.target_orientation.x = 1.0;

    const auto output = FeedForward{0}.update(0.0, 0.0, input);

    EXPECT_DOUBLE_EQ(output.cmd.esc_thrusts[0].value, 0.0);
    EXPECT_DOUBLE_EQ(output.cmd.esc_thrusts[1].value, 1.0);
    EXPECT_DOUBLE_EQ(output.cmd.esc_thrusts[2].value, 1.0);
    EXPECT_DOUBLE_EQ(output.cmd.esc_thrusts[3].value, 0.0);
    EXPECT_NEAR(output.cmd.servo_angles[1].value, 90.0, 1e-12);
    EXPECT_NEAR(output.cmd.servo_angles[2].value, -90.0, 1e-12);
}

}  // namespace sinsei_umiusi_control::test::controller::logic::attitude

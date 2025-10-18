#include "mock/can.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <rcpputils/tl_expected/expected.hpp>

#include "gmock/gmock.h"
#include "sinsei_umiusi_control/hardware_model/can_model.hpp"

namespace succmd = sinsei_umiusi_control::cmd;
// namespace sucutil = sinsei_umiusi_control::util;
namespace suchm = sinsei_umiusi_control::hardware_model;

using sinsei_umiusi_control::test::mock::Can;
using testing::Return;

namespace sinsei_umiusi_control::test::hardware_model::can {

namespace {

constexpr auto _ = testing::_;

constexpr int VESC_ID_1 = 1;
constexpr int VESC_ID_2 = 2;
constexpr int VESC_ID_3 = 3;
constexpr int VESC_ID_4 = 4;
constexpr size_t PERIOD_LED_TAPE_PER_THRUSTERS = 1;
constexpr util::ThrusterMode THRUSTER_MODE = util::ThrusterMode::Can;

#define VESC_IDS \
    { VESC_ID_1, VESC_ID_2, VESC_ID_3, VESC_ID_4 }

}  // namespace

TEST(CanModelTest, CanModelOnInitTest) {
    auto can = std::make_shared<Can>();

    EXPECT_CALL(*can, init(_)).Times(1).WillOnce(Return(tl::expected<void, std::string>{}));

    auto can_model = suchm::CanModel(can, VESC_IDS, PERIOD_LED_TAPE_PER_THRUSTERS, THRUSTER_MODE);
    auto result = can_model.on_init();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
}

TEST(CanModelTest, CanModelOnDestroyTest) {
    auto can = std::make_shared<Can>();

    EXPECT_CALL(*can, close()).Times(1).WillOnce(Return(tl::expected<void, std::string>{}));

    auto can_model = suchm::CanModel(can, VESC_IDS, PERIOD_LED_TAPE_PER_THRUSTERS, THRUSTER_MODE);
    auto result = can_model.on_destroy();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
}

// TODO: `CanModelOnReadTest`を実装する

TEST(CanModelTest, CanModelCanModeOnWriteTest) {
    auto can = std::make_shared<Can>();

    // FIXME: 実装が終わっていないので失敗する
    EXPECT_CALL(*can, send_frame(_))
        .Times(/*1*/ 0)
        .WillOnce(Return(tl::expected<void, std::string>{}));

    // FIXME: 内部状態が変化するので、何回か`on_write`を呼び出してテストする必要がある。(実装が終わったら修正)
    auto can_model = suchm::CanModel(can, VESC_IDS, PERIOD_LED_TAPE_PER_THRUSTERS, THRUSTER_MODE);
    auto result = can_model.on_write(
        succmd::main_power::Enabled{false},
        std::array<succmd::thruster::esc::Enabled, 4>{
            succmd::thruster::esc::Enabled{false}, succmd::thruster::esc::Enabled{false},
            succmd::thruster::esc::Enabled{false}, succmd::thruster::esc::Enabled{false}},
        std::array<succmd::thruster::esc::DutyCycle, 4>{
            succmd::thruster::esc::DutyCycle{1.0f}, succmd::thruster::esc::DutyCycle{0.0f},
            succmd::thruster::esc::DutyCycle{0.0f}, succmd::thruster::esc::DutyCycle{0.0f}},
        std::array<succmd::thruster::servo::Enabled, 4>{
            succmd::thruster::servo::Enabled{false}, succmd::thruster::servo::Enabled{false},
            succmd::thruster::servo::Enabled{false}, succmd::thruster::servo::Enabled{false}},
        std::array<succmd::thruster::servo::Angle, 4>{
            succmd::thruster::servo::Angle{0.0f}, succmd::thruster::servo::Angle{0.0f},
            succmd::thruster::servo::Angle{0.0f}, succmd::thruster::servo::Angle{0.0f}},

        succmd::led_tape::Color{0, 0, 0});
    // FIXME: 実装が終わっていないので失敗する
    ASSERT_TRUE(!result);
    // ASSERT_TRUE(result) << std::string("Error: ") + result.error();
}

TEST(CanModelTest, CanModelDirectModeOnWriteTest) {
    constexpr bool DUMMY_MAIN_POWER_ENABLED = true;
    constexpr uint8_t DUMMY_LED_TAPE_COLOR_R = 255;
    constexpr uint8_t DUMMY_LED_TAPE_COLOR_G = 0;
    constexpr uint8_t DUMMY_LED_TAPE_COLOR_B = 0;

    auto can = std::make_shared<Can>();

    // FIXME: 実装が終わっていないので、`send_frame`の呼び出しがない
    EXPECT_CALL(*can, send_frame(_))
        .Times(/*1*/ 0)
        .WillOnce(Return(tl::expected<void, std::string>{}));

    auto can_model = suchm::CanModel(can, VESC_IDS, PERIOD_LED_TAPE_PER_THRUSTERS, THRUSTER_MODE);
    auto result = can_model.on_write(
        succmd::main_power::Enabled{DUMMY_MAIN_POWER_ENABLED}, succmd::led_tape::Color{
                                                                   DUMMY_LED_TAPE_COLOR_R,
                                                                   DUMMY_LED_TAPE_COLOR_G,
                                                                   DUMMY_LED_TAPE_COLOR_B,
                                                               });
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
}

}  // namespace sinsei_umiusi_control::test::hardware_model::can

#include "mock/can.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <rcpputils/tl_expected/expected.hpp>

#include "gmock/gmock.h"
#include "sinsei_umiusi_control/hardware_model/can/vesc_model.hpp"
#include "sinsei_umiusi_control/hardware_model/can_model.hpp"

namespace succmd = sinsei_umiusi_control::cmd;
namespace sucutil = sinsei_umiusi_control::util;
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

#define VESC_IDS \
    { VESC_ID_1, VESC_ID_2, VESC_ID_3, VESC_ID_4 }

}  // namespace

TEST(CanModelTest, CanModelOnInitTest) {
    auto can = std::make_shared<Can>();

    EXPECT_CALL(*can, init(_)).Times(1).WillOnce(Return(tl::expected<void, std::string>{}));

    auto can_model = suchm::CanModel(can, VESC_IDS);
    auto result = can_model.on_init();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
}

TEST(CanModelTest, CanModelOnDestroyTest) {
    auto can = std::make_shared<Can>();

    EXPECT_CALL(*can, close()).Times(1).WillOnce(Return(tl::expected<void, std::string>{}));

    auto can_model = suchm::CanModel(can, VESC_IDS);
    auto result = can_model.on_destroy();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
}

TEST(CanModelTest, CanModelOnReadTest) {
    constexpr uint32_t DUMMY_FRAME_ID_RECV =
        (static_cast<uint32_t>(suchm::can::VescStatusCommandID::CAN_PACKET_STATUS) << 8) |
        VESC_ID_1;
    constexpr std::array<std::byte, 8> DUMMY_FRAME_DATA_RECV = {};

    auto can = std::make_shared<Can>();

    // TODO: parameterlize
    EXPECT_CALL(*can, recv_frame())
        .Times(1)
        .WillOnce(Return(tl::expected<suchm::interface::CanFrame, std::string>{
            suchm::interface::CanFrame{DUMMY_FRAME_ID_RECV, 8, DUMMY_FRAME_DATA_RECV, true}}));

    auto can_model = suchm::CanModel(can, VESC_IDS);
    auto result = can_model.on_read();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
}

TEST(CanModelTest, CanModelCanModeOnWriteTest) {
    auto can = std::make_shared<Can>();

    EXPECT_CALL(*can, send_frame(_)).Times(1).WillOnce(Return(tl::expected<void, std::string>{}));

    // TODO: parameterlize
    auto can_model = suchm::CanModel(can, VESC_IDS);
    auto result = can_model.on_write(
        succmd::main_power::Enabled{false},
        std::array<succmd::thruster::EscEnabled, 4>{
            succmd::thruster::EscEnabled{false}, succmd::thruster::EscEnabled{false},
            succmd::thruster::EscEnabled{false}, succmd::thruster::EscEnabled{false}},
        std::array<succmd::thruster::ServoEnabled, 4>{
            succmd::thruster::ServoEnabled{false}, succmd::thruster::ServoEnabled{false},
            succmd::thruster::ServoEnabled{false}, succmd::thruster::ServoEnabled{false}},
        std::array<succmd::thruster::DutyCycle, 4>{
            succmd::thruster::DutyCycle{1.0f}, succmd::thruster::DutyCycle{0.0f},
            succmd::thruster::DutyCycle{0.0f}, succmd::thruster::DutyCycle{0.0f}},
        std::array<succmd::thruster::Angle, 4>{
            succmd::thruster::Angle{0.0f}, succmd::thruster::Angle{0.0f},
            succmd::thruster::Angle{0.0f}, succmd::thruster::Angle{0.0f}},

        succmd::led_tape::Color{0, 0, 0});
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
}

TEST(CanModelTest, CanModelDirectModeOnWriteTest) {
    auto can = std::make_shared<Can>();

    // FIXME: 実装が終わっていないので、`send_frame`の呼び出しがない
    EXPECT_CALL(*can, send_frame(_))
        .Times(/*1*/ 0)
        .WillOnce(Return(tl::expected<void, std::string>{}));

    // TODO: parameterlize
    auto can_model = suchm::CanModel(can, VESC_IDS);
    auto result =
        can_model.on_write(succmd::main_power::Enabled{true}, succmd::led_tape::Color{255, 0, 0});
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
}

}  // namespace sinsei_umiusi_control::test::hardware_model::can

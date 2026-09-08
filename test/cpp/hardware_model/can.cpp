#include "mock/can.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>
#include <vector>

#include "gmock/gmock.h"
#include "sinsei_umiusi_control/cmd/thruster/esc.hpp"
#include "sinsei_umiusi_control/hardware_model/can_model.hpp"
#include "sinsei_umiusi_control/util/byte.hpp"

namespace succmd = sinsei_umiusi_control::cmd;
// namespace sucutil = sinsei_umiusi_control::util;
namespace suchm = sinsei_umiusi_control::hardware_model;

using sinsei_umiusi_control::test::mock::Can;
using testing::Invoke;
using testing::Return;

namespace sinsei_umiusi_control::test::hardware_model::can {

namespace {

constexpr auto _ = testing::_;

constexpr int VESC_ID_1 = 1;
constexpr int VESC_ID_2 = 2;
constexpr int VESC_ID_3 = 3;
constexpr int VESC_ID_4 = 4;
constexpr size_t PERIOD_LED_TAPE_PER_THRUSTERS = 1;

const auto THRUSTERS = std::vector<suchm::CanModel::ThrusterConfig>{
    {"thruster1", VESC_ID_1},
    {"thruster2", VESC_ID_2},
    {"thruster3", VESC_ID_3},
    {"thruster4", VESC_ID_4},
};

auto make_thrusters(size_t count) -> std::vector<suchm::CanModel::ThrusterConfig> {
    auto thrusters = std::vector<suchm::CanModel::ThrusterConfig>{};
    thrusters.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        thrusters.push_back(suchm::CanModel::ThrusterConfig{
            "thruster" + std::to_string(i + 1), static_cast<uint8_t>(i + 1)});
    }
    return thrusters;
}

auto make_enabled_command(const std::string & name, double duty_cycle = 0.25)
    -> suchm::CanModel::ThrusterCommand {
    return suchm::CanModel::ThrusterCommand{
        name,
        succmd::thruster::esc::Allowed{true},
        succmd::thruster::esc::DutyCycle{duty_cycle},
        succmd::thruster::servo::Allowed{true},
        succmd::thruster::servo::Angle{0.0},
    };
}

auto make_enabled_commands(size_t count) -> std::vector<suchm::CanModel::ThrusterCommand> {
    auto commands = std::vector<suchm::CanModel::ThrusterCommand>{};
    commands.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        const auto name = "thruster" + std::to_string(i + 1);
        commands.push_back(make_enabled_command(name));
    }
    return commands;
}

auto make_vesc_status_frame(
    uint8_t vesc_id, uint32_t command_id, suchm::interface::CanFrame::Data data = {}) {
    return suchm::interface::CanFrame{
        (static_cast<suchm::interface::CanFrame::Id>(command_id) << 8) | vesc_id,
        8,
        data,
        true,
    };
}

}  // namespace

TEST(CanModelTest, CanModelOnInitTest) {
    auto can = std::make_shared<Can>();

    EXPECT_CALL(*can, init(_)).Times(1).WillOnce(Return(tl::expected<void, std::string>{}));

    auto can_model = suchm::CanModel(can, THRUSTERS, PERIOD_LED_TAPE_PER_THRUSTERS);
    auto result = can_model.on_init();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
}

TEST(CanModelTest, CanModelOnDestroyTest) {
    auto can = std::make_shared<Can>();

    EXPECT_CALL(*can, close()).Times(1).WillOnce(Return(tl::expected<void, std::string>{}));

    auto can_model = suchm::CanModel(can, THRUSTERS, PERIOD_LED_TAPE_PER_THRUSTERS);
    auto result = can_model.on_destroy();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
}

// TODO: `can::MainPowerModel`を追加したら、MainPowerModel向けの`on_read`テストケースも追加する
TEST(CanModelTest, CanModelOnReadTimeoutReturnsTimeoutErrorTest) {
    auto can = std::make_shared<Can>();

    EXPECT_CALL(*can, recv_frame())
        .Times(1)
        .WillOnce(Return(
            tl::expected<std::optional<suchm::interface::CanFrame>, std::string>{std::nullopt}));

    auto can_model = suchm::CanModel(can, THRUSTERS, PERIOD_LED_TAPE_PER_THRUSTERS);
    const auto result = can_model.on_read();
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), "CAN read timeout: no CAN frame received within the timeout period");
}

TEST(CanModelTest, CanModelOnReadPacketStatusReturnsRpmUpdateTest) {
    auto can = std::make_shared<Can>();

    const auto frame = make_vesc_status_frame(
        VESC_ID_1, suchm::can::PacketStatus::ID,
        {std::byte{0x00}, std::byte{0x00}, std::byte{0x05}, std::byte{0x78}, std::byte{0x00},
         std::byte{0x7B}, std::byte{0x01}, std::byte{0xF4}});

    EXPECT_CALL(*can, recv_frame())
        .Times(1)
        .WillOnce(
            Return(tl::expected<std::optional<suchm::interface::CanFrame>, std::string>{frame}));

    auto can_model = suchm::CanModel(can, THRUSTERS, PERIOD_LED_TAPE_PER_THRUSTERS);
    const auto result = can_model.on_read();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
    const auto variant = result.value();
    ASSERT_EQ(variant.index(), 0u);
    const auto & [name, rpm] = std::get<0>(variant);
    EXPECT_EQ(name, "thruster1");
    EXPECT_DOUBLE_EQ(rpm.value, 200.0);
}

TEST(CanModelTest, CanModelOnReadUnsupportedPacketStatusReturnsErrorTest) {
    auto can = std::make_shared<Can>();

    const auto frame = make_vesc_status_frame(VESC_ID_1, suchm::can::PacketStatus2::ID);

    EXPECT_CALL(*can, recv_frame())
        .Times(1)
        .WillOnce(
            Return(tl::expected<std::optional<suchm::interface::CanFrame>, std::string>{frame}));

    auto can_model = suchm::CanModel(can, THRUSTERS, PERIOD_LED_TAPE_PER_THRUSTERS);
    const auto result = can_model.on_read();
    ASSERT_FALSE(result);
    EXPECT_EQ(
        result.error(),
        "Unsupported VESC packet status variant received (thruster 'thruster1' (VESC 1), "
        "variant index: 1)");
}

TEST(CanModelTest, CanModelOnReadUnhandledFrameReturnsErrorTest) {
    auto can = std::make_shared<Can>();

    // TODO: `can::MainPowerModel`を追加したら、このフレームがhandledになるか見直す
    const auto frame = make_vesc_status_frame(0x21, suchm::can::PacketStatus::ID);

    EXPECT_CALL(*can, recv_frame())
        .Times(1)
        .WillOnce(
            Return(tl::expected<std::optional<suchm::interface::CanFrame>, std::string>{frame}));

    auto can_model = suchm::CanModel(can, THRUSTERS, PERIOD_LED_TAPE_PER_THRUSTERS);
    const auto result = can_model.on_read();
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), "Unhandled CAN frame: no registered model accepted frame id 2337");
}

TEST(CanModelTest, CanModelOnInitRejectsEmptyThrusterConfigurationTest) {
    auto can = std::make_shared<Can>();

    EXPECT_CALL(*can, init(_)).Times(0);

    auto can_model = suchm::CanModel(can, {}, PERIOD_LED_TAPE_PER_THRUSTERS);
    const auto result = can_model.on_init();
    ASSERT_FALSE(result);
    EXPECT_EQ(
        result.error(), "Invalid thruster configuration: At least one thruster must be configured");
}

TEST(CanModelTest, CanModelOnInitRejectsDuplicateThrusterNameTest) {
    auto can = std::make_shared<Can>();
    auto thrusters = THRUSTERS;
    thrusters[1].name = thrusters[0].name;

    EXPECT_CALL(*can, init(_)).Times(0);

    auto can_model = suchm::CanModel(can, thrusters, PERIOD_LED_TAPE_PER_THRUSTERS);
    const auto result = can_model.on_init();
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), "Invalid thruster configuration: Duplicate thruster name: thruster1");
}

TEST(CanModelTest, CanModelOnInitRejectsDuplicateVescIdTest) {
    auto can = std::make_shared<Can>();
    auto thrusters = THRUSTERS;
    thrusters[1].vesc_id = thrusters[0].vesc_id;

    EXPECT_CALL(*can, init(_)).Times(0);

    auto can_model = suchm::CanModel(can, thrusters, PERIOD_LED_TAPE_PER_THRUSTERS);
    const auto result = can_model.on_init();
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), "Invalid thruster configuration: Duplicate VESC ID: 1");
}

TEST(CanModelTest, OnWriteUsesThrusterNamesInsteadOfCommandOrder) {
    auto can = std::make_shared<Can>();
    auto sent_frames = std::vector<suchm::interface::CanFrame>{};

    EXPECT_CALL(*can, init(_)).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*can, send_frame(_))
        .Times(3)
        .WillRepeatedly(Invoke(
            [&sent_frames](
                const suchm::interface::CanFrame & frame) -> tl::expected<void, std::string> {
                sent_frames.push_back(frame);
                return {};
            }));

    constexpr size_t THRUSTER_COUNT = 3;
    constexpr size_t PERIOD_WITHOUT_LED_IN_FIRST_CYCLE = 10;
    auto can_model =
        suchm::CanModel(can, make_thrusters(THRUSTER_COUNT), PERIOD_WITHOUT_LED_IN_FIRST_CYCLE);
    ASSERT_TRUE(can_model.on_init());

    // 設定順とは逆に並べ、名前で対応付けられることを確認する
    const auto thruster_commands = std::vector<suchm::CanModel::ThrusterCommand>{
        make_enabled_command("thruster3", 0.75),
        make_enabled_command("thruster2", 0.50),
        make_enabled_command("thruster1", 0.25),
    };

    // 最初の2回は未実装のESC許可コマンドで、続く3回が各スラスタのDutyコマンドになる
    for (size_t i = 0; i < 5; ++i) {
        can_model.on_write(
            succmd::main_power::Enabled{false}, thruster_commands,
            succmd::led_tape::Color{0, 0, 0});
    }

    ASSERT_EQ(sent_frames.size(), THRUSTER_COUNT);
    EXPECT_EQ(sent_frames[0].id, VESC_ID_1);
    EXPECT_EQ(sent_frames[1].id, VESC_ID_2);
    EXPECT_EQ(sent_frames[2].id, VESC_ID_3);
    EXPECT_EQ(sinsei_umiusi_control::util::to_int32_be(sent_frames[0].data).value(), 25000);
    EXPECT_EQ(sinsei_umiusi_control::util::to_int32_be(sent_frames[1].data).value(), 50000);
    EXPECT_EQ(sinsei_umiusi_control::util::to_int32_be(sent_frames[2].data).value(), 75000);
}

TEST(CanModelTest, OnWriteRejectsUnknownThrusterName) {
    auto can = std::make_shared<Can>();

    EXPECT_CALL(*can, init(_)).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*can, send_frame(_)).Times(0);

    constexpr size_t THRUSTER_COUNT = 3;
    auto can_model =
        suchm::CanModel(can, make_thrusters(THRUSTER_COUNT), PERIOD_LED_TAPE_PER_THRUSTERS);
    ASSERT_TRUE(can_model.on_init());

    auto thruster_commands = make_enabled_commands(THRUSTER_COUNT);
    thruster_commands.back().name = "unknown";
    const auto result = can_model.on_write(
        succmd::main_power::Enabled{false}, thruster_commands, succmd::led_tape::Color{0, 0, 0});

    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), "Invalid thruster commands: Unknown thruster name: unknown");
}

TEST(CanModelTest, OnWriteRejectsDuplicateThrusterCommand) {
    auto can = std::make_shared<Can>();

    EXPECT_CALL(*can, init(_)).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*can, send_frame(_)).Times(0);

    constexpr size_t THRUSTER_COUNT = 3;
    auto can_model =
        suchm::CanModel(can, make_thrusters(THRUSTER_COUNT), PERIOD_LED_TAPE_PER_THRUSTERS);
    ASSERT_TRUE(can_model.on_init());

    auto thruster_commands = make_enabled_commands(THRUSTER_COUNT);
    thruster_commands.back().name = thruster_commands.front().name;
    const auto result = can_model.on_write(
        succmd::main_power::Enabled{false}, thruster_commands, succmd::led_tape::Color{0, 0, 0});

    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), "Invalid thruster commands: Duplicate thruster command: thruster1");
}

class CanModelVariableThrusterCountTest : public testing::TestWithParam<size_t> {};

TEST_P(CanModelVariableThrusterCountTest, WritesEachDynamicThrusterInRoundRobinOrder) {
    const auto thruster_count = GetParam();
    auto can = std::make_shared<Can>();
    auto sent_frame_ids = std::vector<suchm::interface::CanFrame::Id>{};

    EXPECT_CALL(*can, send_frame(_))
        .Times(2 * thruster_count)
        .WillRepeatedly(Invoke(
            [&sent_frame_ids](
                const suchm::interface::CanFrame & frame) -> tl::expected<void, std::string> {
                sent_frame_ids.push_back(frame.id);
                return {};
            }));

    constexpr size_t PERIOD_WITHOUT_LED_IN_FIRST_CYCLE = 10;
    auto can_model =
        suchm::CanModel(can, make_thrusters(thruster_count), PERIOD_WITHOUT_LED_IN_FIRST_CYCLE);
    EXPECT_CALL(*can, init(_)).WillOnce(Return(tl::expected<void, std::string>{}));
    ASSERT_TRUE(can_model.on_init());

    auto thruster_commands = make_enabled_commands(thruster_count);
    // 指令の並び順がスラスタの設定順に依存しないことも確認する
    std::reverse(thruster_commands.begin(), thruster_commands.end());

    constexpr size_t COMMAND_TYPES_PER_THRUSTER = 4;
    // FIXME: ESC・サーボのallowedコマンドは未実装のため、Duty比とサーボ角度の送信だけが成功する
    for (size_t i = 0; i < thruster_count * COMMAND_TYPES_PER_THRUSTER; ++i) {
        const auto result = can_model.on_write(
            succmd::main_power::Enabled{false}, thruster_commands,
            succmd::led_tape::Color{0, 0, 0});
        const auto loop_count = i + 1;
        const auto writes_duty = loop_count >= thruster_count && loop_count < 2 * thruster_count;
        const auto writes_servo =
            loop_count >= 3 * thruster_count && loop_count < 4 * thruster_count;
        EXPECT_EQ(result.has_value(), writes_duty || writes_servo);
    }

    auto expected_frame_ids = std::vector<suchm::interface::CanFrame::Id>{};
    expected_frame_ids.reserve(2 * thruster_count);
    for (size_t i = 0; i < thruster_count; ++i) {
        expected_frame_ids.push_back(static_cast<uint8_t>(i + 1));
    }
    for (size_t i = 0; i < thruster_count; ++i) {
        expected_frame_ids.push_back(
            (static_cast<suchm::interface::CanFrame::Id>(
                 suchm::can::VescSimpleCommandID::CAN_PACKET_SET_SERVO)
             << 8) |
            static_cast<uint8_t>(i + 1));
    }
    EXPECT_EQ(sent_frame_ids, expected_frame_ids);
}

TEST_P(CanModelVariableThrusterCountTest, RoutesReceivedStatusToConfiguredThrusterName) {
    const auto thruster_count = GetParam();
    auto can = std::make_shared<Can>();
    const auto vesc_id = static_cast<uint8_t>(thruster_count);
    const auto frame = make_vesc_status_frame(
        vesc_id, suchm::can::PacketStatus::ID,
        {std::byte{0x00}, std::byte{0x00}, std::byte{0x05}, std::byte{0x78}, std::byte{0x00},
         std::byte{0x7B}, std::byte{0x01}, std::byte{0xF4}});

    EXPECT_CALL(*can, recv_frame())
        .Times(1)
        .WillOnce(
            Return(tl::expected<std::optional<suchm::interface::CanFrame>, std::string>{frame}));

    auto can_model =
        suchm::CanModel(can, make_thrusters(thruster_count), PERIOD_LED_TAPE_PER_THRUSTERS);
    const auto result = can_model.on_read();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
    const auto & [name, rpm] = std::get<0>(result.value());
    EXPECT_EQ(name, "thruster" + std::to_string(thruster_count));
    EXPECT_DOUBLE_EQ(rpm.value, 200.0);
}

INSTANTIATE_TEST_SUITE_P(
    ThrusterCounts, CanModelVariableThrusterCountTest, testing::Values(1u, 3u, 4u, 6u));

}  // namespace sinsei_umiusi_control::test::hardware_model::can

#include "mock/can.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

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

constexpr uint8_t VESC_ID_MAX = 127;
constexpr uint8_t HARMONY_BMS_ID = 3;

auto make_thrusters(size_t count) -> std::vector<suchm::CanModel::ThrusterConfig> {
    auto thrusters = std::vector<suchm::CanModel::ThrusterConfig>{};
    thrusters.reserve(count);
    const auto first_vesc_id = static_cast<size_t>(VESC_ID_MAX) + 1 - count;
    for (size_t i = 0; i < count; ++i) {
        thrusters.push_back(suchm::CanModel::ThrusterConfig{
            "thruster" + std::to_string(i + 1), static_cast<uint8_t>(first_vesc_id + i)});
    }
    return thrusters;
}

const auto THRUSTERS = make_thrusters(4);

auto make_enabled_command(double duty_cycle = 0.25) -> suchm::CanModel::ThrusterCommand {
    return suchm::CanModel::ThrusterCommand{
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
        commands.push_back(make_enabled_command());
    }
    return commands;
}

template <typename PacketId>
auto make_vesc_status_frame(
    uint8_t vesc_id, PacketId packet_id, suchm::interface::CanFrame::Data data = {}) {
    return suchm::interface::CanFrame{
        (static_cast<suchm::interface::CanFrame::Id>(packet_id) << 8) | vesc_id,
        8,
        data,
        true,
    };
}

}  // namespace

TEST(CanModelTest, CanModelOnInitTest) {
    auto can = std::make_shared<Can>();

    EXPECT_CALL(*can, init(_)).Times(1).WillOnce(Return(tl::expected<void, std::string>{}));

    auto can_model = suchm::CanModel(can, THRUSTERS, HARMONY_BMS_ID);
    auto result = can_model.on_init();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
}

TEST(CanModelTest, CanModelOnDestroyTest) {
    auto can = std::make_shared<Can>();

    EXPECT_CALL(*can, close()).Times(1).WillOnce(Return(tl::expected<void, std::string>{}));

    auto can_model = suchm::CanModel(can, THRUSTERS, HARMONY_BMS_ID);
    auto result = can_model.on_destroy();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
}

TEST(CanModelTest, CanModelOnReadNoFrameReturnsNoUpdateTest) {
    auto can = std::make_shared<Can>();

    EXPECT_CALL(*can, recv_frames())
        .Times(1)
        .WillOnce(Return(tl::expected<std::vector<suchm::interface::CanFrame>, std::string>{
            std::vector<suchm::interface::CanFrame>{}}));

    auto can_model = suchm::CanModel(can, THRUSTERS, HARMONY_BMS_ID);
    const auto result = can_model.on_read();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
    EXPECT_TRUE(result.value().states.empty());
    EXPECT_TRUE(result.value().error_message.empty());
}

TEST(CanModelTest, CanModelOnReadPacketStatusReturnsRpmUpdateTest) {
    auto can = std::make_shared<Can>();

    const auto frame = make_vesc_status_frame(
        THRUSTERS[0].vesc_id, suchm::can::VescModel::PacketStatus::ID,
        {std::byte{0x00}, std::byte{0x00}, std::byte{0x05}, std::byte{0x78}, std::byte{0x00},
         std::byte{0x7B}, std::byte{0x01}, std::byte{0xF4}});

    EXPECT_CALL(*can, recv_frames())
        .Times(1)
        .WillOnce(
            Return(tl::expected<std::vector<suchm::interface::CanFrame>, std::string>{{frame}}));

    auto can_model = suchm::CanModel(can, THRUSTERS, HARMONY_BMS_ID);
    const auto result = can_model.on_read();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
    ASSERT_EQ(result.value().states.size(), 1u);
    EXPECT_TRUE(result.value().error_message.empty());
    const auto & state = result.value().states[0];
    ASSERT_EQ(state.index(), 0u);
    const auto & [name, rpm] = std::get<0>(state);
    EXPECT_EQ(name, "thruster1");
    EXPECT_DOUBLE_EQ(rpm.value, 200.0);
}

TEST(CanModelTest, CanModelOnReadRoutesHarmonyBmsFrameTest) {
    auto can = std::make_shared<Can>();
    const auto frame = make_vesc_status_frame(
        HARMONY_BMS_ID, suchm::can::HarmonyBmsModel::PacketId::Summary,
        {std::byte{0x0E}, std::byte{0x74}, std::byte{0x10}, std::byte{0x04}, std::byte{0x80},
         std::byte{0xFF}, std::byte{42}, std::byte{0x17}});

    EXPECT_CALL(*can, recv_frames())
        .Times(1)
        .WillOnce(
            Return(tl::expected<std::vector<suchm::interface::CanFrame>, std::string>{{frame}}));

    auto can_model = suchm::CanModel(can, THRUSTERS, HARMONY_BMS_ID);
    const auto result = can_model.on_read();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
    ASSERT_EQ(result.value().states.size(), 1u);
    EXPECT_TRUE(result.value().error_message.empty());
    const auto & bms = std::get<suchm::can::HarmonyBmsModel::State>(result.value().states[0]);
    EXPECT_DOUBLE_EQ(bms.cell_voltage_range.min, 3.7);
    EXPECT_DOUBLE_EQ(bms.cell_voltage_range.max, 4.1);
    EXPECT_TRUE(bms.status.charging);
}

TEST(CanModelTest, CanModelOnReadProcessesAllReceivedFramesTest) {
    auto can = std::make_shared<Can>();

    const auto data = suchm::interface::CanFrame::Data{
        std::byte{0x00}, std::byte{0x00}, std::byte{0x05}, std::byte{0x78},
        std::byte{0x00}, std::byte{0x7B}, std::byte{0x01}, std::byte{0xF4}};
    const auto frame1 =
        make_vesc_status_frame(THRUSTERS[0].vesc_id, suchm::can::VescModel::PacketStatus::ID, data);
    const auto frame2 =
        make_vesc_status_frame(THRUSTERS[1].vesc_id, suchm::can::VescModel::PacketStatus::ID, data);

    EXPECT_CALL(*can, recv_frames())
        .Times(1)
        .WillOnce(Return(
            tl::expected<std::vector<suchm::interface::CanFrame>, std::string>{{frame1, frame2}}));

    auto can_model = suchm::CanModel(can, THRUSTERS, HARMONY_BMS_ID);
    const auto result = can_model.on_read();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
    ASSERT_EQ(result.value().states.size(), 2u);
    EXPECT_TRUE(result.value().error_message.empty());

    const auto & [name1, rpm1] = std::get<0>(result.value().states[0]);
    const auto & [name2, rpm2] = std::get<0>(result.value().states[1]);
    EXPECT_EQ(name1, "thruster1");
    EXPECT_EQ(name2, "thruster2");
    EXPECT_DOUBLE_EQ(rpm1.value, 200.0);
    EXPECT_DOUBLE_EQ(rpm2.value, 200.0);
}

TEST(CanModelTest, CanModelOnReadUsesUnusedStatusAsHeartbeatTest) {
    auto can = std::make_shared<Can>();

    const auto data = suchm::interface::CanFrame::Data{
        std::byte{0x00}, std::byte{0x00}, std::byte{0x05}, std::byte{0x78},
        std::byte{0x00}, std::byte{0x7B}, std::byte{0x01}, std::byte{0xF4}};
    const auto frame1 =
        make_vesc_status_frame(THRUSTERS[0].vesc_id, suchm::can::VescModel::PacketStatus::ID, data);
    const auto unused_frame =
        make_vesc_status_frame(THRUSTERS[0].vesc_id, suchm::can::VescModel::PacketStatus2::ID);
    const auto frame2 =
        make_vesc_status_frame(THRUSTERS[1].vesc_id, suchm::can::VescModel::PacketStatus::ID, data);

    EXPECT_CALL(*can, recv_frames())
        .Times(1)
        .WillOnce(Return(tl::expected<std::vector<suchm::interface::CanFrame>, std::string>{
            {frame1, unused_frame, frame2}}));

    auto can_model = suchm::CanModel(can, THRUSTERS, HARMONY_BMS_ID);
    const auto result = can_model.on_read();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
    ASSERT_EQ(result.value().states.size(), 3u);
    EXPECT_TRUE(result.value().error_message.empty());

    const auto & [name1, rpm1] = std::get<0>(result.value().states[0]);
    const auto & [heartbeat_name, heartbeat] = std::get<3>(result.value().states[1]);
    const auto & [name2, rpm2] = std::get<0>(result.value().states[2]);
    EXPECT_EQ(name1, "thruster1");
    EXPECT_EQ(name2, "thruster2");
    EXPECT_DOUBLE_EQ(rpm1.value, 200.0);
    EXPECT_DOUBLE_EQ(rpm2.value, 200.0);
    EXPECT_EQ(heartbeat_name, "thruster1");
    EXPECT_TRUE(heartbeat.is_ok);
}

TEST(CanModelTest, CanModelOnReadUnusedPacketStatusReturnsHeartbeatTest) {
    auto can = std::make_shared<Can>();

    const auto frame =
        make_vesc_status_frame(THRUSTERS[0].vesc_id, suchm::can::VescModel::PacketStatus2::ID);

    EXPECT_CALL(*can, recv_frames())
        .Times(1)
        .WillOnce(
            Return(tl::expected<std::vector<suchm::interface::CanFrame>, std::string>{{frame}}));

    auto can_model = suchm::CanModel(can, THRUSTERS, HARMONY_BMS_ID);
    const auto result = can_model.on_read();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
    ASSERT_EQ(result.value().states.size(), 1u);
    EXPECT_TRUE(result.value().error_message.empty());
    const auto & [name, heartbeat] = std::get<3>(result.value().states[0]);
    EXPECT_EQ(name, "thruster1");
    EXPECT_TRUE(heartbeat.is_ok);
}

TEST(CanModelTest, CanModelOnReadUndecodableFrameReturnsErrorTest) {
    auto can = std::make_shared<Can>();

    const auto frame = make_vesc_status_frame(0x21, suchm::can::VescModel::PacketStatus::ID);

    EXPECT_CALL(*can, recv_frames())
        .Times(1)
        .WillOnce(
            Return(tl::expected<std::vector<suchm::interface::CanFrame>, std::string>{{frame}}));

    auto can_model = suchm::CanModel(can, THRUSTERS, HARMONY_BMS_ID);
    const auto result = can_model.on_read();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
    EXPECT_TRUE(result.value().states.empty());
    EXPECT_EQ(
        result.value().error_message,
        "Failed to decode CAN frame: no registered model accepted frame id 2337");
}

TEST(CanModelTest, CanModelOnReadIgnoresVescTransportFramesTest) {
    auto can = std::make_shared<Can>();

    const auto frames = std::vector<suchm::interface::CanFrame>{
        make_vesc_status_frame(
            HARMONY_BMS_ID, suchm::can::VescModel::PacketId::CAN_PACKET_FILL_RX_BUFFER),
        make_vesc_status_frame(
            HARMONY_BMS_ID, suchm::can::VescModel::PacketId::CAN_PACKET_FILL_RX_BUFFER_LONG),
        make_vesc_status_frame(
            HARMONY_BMS_ID, suchm::can::VescModel::PacketId::CAN_PACKET_PROCESS_RX_BUFFER),
        make_vesc_status_frame(
            HARMONY_BMS_ID, suchm::can::VescModel::PacketId::CAN_PACKET_PROCESS_SHORT_BUFFER),
    };

    EXPECT_CALL(*can, recv_frames())
        .Times(1)
        .WillOnce(
            Return(tl::expected<std::vector<suchm::interface::CanFrame>, std::string>{frames}));

    auto can_model = suchm::CanModel(can, THRUSTERS, HARMONY_BMS_ID);
    const auto result = can_model.on_read();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
    EXPECT_TRUE(result.value().states.empty());
    EXPECT_TRUE(result.value().error_message.empty());
}

TEST(CanModelTest, CanModelOnReadRejectsUnknownPacketFromRegisteredNodeTest) {
    auto can = std::make_shared<Can>();

    const auto frame = make_vesc_status_frame(
        HARMONY_BMS_ID, static_cast<suchm::can::HarmonyBmsModel::PacketId>(0x7F));

    EXPECT_CALL(*can, recv_frames())
        .Times(1)
        .WillOnce(
            Return(tl::expected<std::vector<suchm::interface::CanFrame>, std::string>{{frame}}));

    auto can_model = suchm::CanModel(can, THRUSTERS, HARMONY_BMS_ID);
    const auto result = can_model.on_read();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
    EXPECT_TRUE(result.value().states.empty());
    EXPECT_EQ(
        result.value().error_message,
        "Failed to decode CAN frame \"32515\" in all models: \n"
        "    Harmony BMS: Harmony BMS received unknown packet ID: 127\n");
}

TEST(CanModelTest, CanModelOnInitRejectsEmptyThrusterConfigurationTest) {
    auto can = std::make_shared<Can>();

    EXPECT_CALL(*can, init(_)).Times(0);

    auto can_model = suchm::CanModel(can, {}, HARMONY_BMS_ID);
    const auto result = can_model.on_init();
    ASSERT_FALSE(result);
    EXPECT_EQ(
        result.error(), "Invalid CAN node configuration: At least one thruster must be configured");
}

TEST(CanModelTest, CanModelOnInitRejectsDuplicateThrusterNameTest) {
    auto can = std::make_shared<Can>();
    auto thrusters = THRUSTERS;
    thrusters[1].name = thrusters[0].name;

    EXPECT_CALL(*can, init(_)).Times(0);

    auto can_model = suchm::CanModel(can, thrusters, HARMONY_BMS_ID);
    const auto result = can_model.on_init();
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), "Invalid CAN node configuration: Duplicate thruster name: thruster1");
}

TEST(CanModelTest, CanModelOnInitRejectsDuplicateVescIdTest) {
    auto can = std::make_shared<Can>();
    auto thrusters = THRUSTERS;
    thrusters[1].vesc_id = thrusters[0].vesc_id;

    EXPECT_CALL(*can, init(_)).Times(0);

    auto can_model = suchm::CanModel(can, thrusters, HARMONY_BMS_ID);
    const auto result = can_model.on_init();
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), "Invalid CAN node configuration: Duplicate VESC ID: 124");
}

TEST(CanModelTest, CanModelOnInitRejectsHarmonyBmsIdConflictingWithVescTest) {
    auto can = std::make_shared<Can>();

    EXPECT_CALL(*can, init(_)).Times(0);

    auto can_model = suchm::CanModel(can, THRUSTERS, THRUSTERS[1].vesc_id);
    const auto result = can_model.on_init();
    ASSERT_FALSE(result);
    EXPECT_EQ(
        result.error(),
        "Invalid CAN node configuration: Harmony BMS ID conflicts with VESC ID: 125");
}

// FIXME: 分電盤のSTM32向けCAN仕様が未確定のため、電源制御を含むon_writeは未テスト
TEST(CanModelTest, OnWriteUsesCommandIndexForConfiguredThrusterOrder) {
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
    const auto thrusters = make_thrusters(THRUSTER_COUNT);
    auto can_model = suchm::CanModel(can, thrusters, HARMONY_BMS_ID);
    ASSERT_TRUE(can_model.on_init());

    // コマンドのインデックスがスラスタ設定のインデックスに対応することを確認する
    const auto thruster_commands = std::vector<suchm::CanModel::ThrusterCommand>{
        make_enabled_command(0.25),
        make_enabled_command(0.50),
        make_enabled_command(0.75),
    };

    // 最初の送信フェーズで各スラスタのDutyをまとめて送信する
    const auto result = can_model.on_write(
        succmd::power_distribution::Enabled{false}, thruster_commands,
        succmd::led_tape::Color{0, 0, 0});
    ASSERT_TRUE(result) << result.error();

    ASSERT_EQ(sent_frames.size(), THRUSTER_COUNT);
    EXPECT_EQ(sent_frames[0].id, thrusters[0].vesc_id);
    EXPECT_EQ(sent_frames[1].id, thrusters[1].vesc_id);
    EXPECT_EQ(sent_frames[2].id, thrusters[2].vesc_id);
    EXPECT_EQ(sinsei_umiusi_control::util::to_int32_be(sent_frames[0].data), 25000);
    EXPECT_EQ(sinsei_umiusi_control::util::to_int32_be(sent_frames[1].data), 50000);
    EXPECT_EQ(sinsei_umiusi_control::util::to_int32_be(sent_frames[2].data), 75000);
}

TEST(CanModelTest, OnWriteRejectsMismatchedThrusterCommandCount) {
    auto can = std::make_shared<Can>();

    EXPECT_CALL(*can, init(_)).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*can, send_frame(_)).Times(0);

    constexpr size_t THRUSTER_COUNT = 3;
    auto can_model = suchm::CanModel(can, make_thrusters(THRUSTER_COUNT), HARMONY_BMS_ID);
    ASSERT_TRUE(can_model.on_init());

    auto thruster_commands = make_enabled_commands(THRUSTER_COUNT);
    thruster_commands.pop_back();
    const auto result = can_model.on_write(
        succmd::power_distribution::Enabled{false}, thruster_commands,
        succmd::led_tape::Color{0, 0, 0});

    ASSERT_FALSE(result);
    EXPECT_EQ(
        result.error(), "Thruster command count does not match configuration: expected 3, got 2");
}

class CanModelVariableThrusterCountTest : public testing::TestWithParam<size_t> {};

TEST_P(CanModelVariableThrusterCountTest, WritesEachDynamicThrusterInAlternatingBatches) {
    const auto thruster_count = GetParam();
    auto can = std::make_shared<Can>();
    auto sent_frame_ids = std::vector<suchm::interface::CanFrame::Id>{};
    const auto thrusters = make_thrusters(thruster_count);

    EXPECT_CALL(*can, send_frame(_))
        .Times(static_cast<int>(2 * thruster_count))
        .WillRepeatedly(Invoke(
            [&sent_frame_ids](
                const suchm::interface::CanFrame & frame) -> tl::expected<void, std::string> {
                sent_frame_ids.push_back(frame.id);
                return {};
            }));

    auto can_model = suchm::CanModel(can, thrusters, HARMONY_BMS_ID);
    EXPECT_CALL(*can, init(_)).WillOnce(Return(tl::expected<void, std::string>{}));
    ASSERT_TRUE(can_model.on_init());

    auto thruster_commands = make_enabled_commands(thruster_count);

    const auto duty_result = can_model.on_write(
        succmd::power_distribution::Enabled{false}, thruster_commands,
        succmd::led_tape::Color{0, 0, 0});
    ASSERT_TRUE(duty_result) << duty_result.error();

    const auto servo_result = can_model.on_write(
        succmd::power_distribution::Enabled{false}, thruster_commands,
        succmd::led_tape::Color{0, 0, 0});
    ASSERT_TRUE(servo_result) << servo_result.error();

    auto expected_frame_ids = std::vector<suchm::interface::CanFrame::Id>{};
    expected_frame_ids.reserve(2 * thruster_count);
    for (size_t i = 0; i < thruster_count; ++i) {
        expected_frame_ids.push_back(thrusters[i].vesc_id);
    }
    for (size_t i = 0; i < thruster_count; ++i) {
        expected_frame_ids.push_back(
            (static_cast<suchm::interface::CanFrame::Id>(
                 suchm::can::VescModel::PacketId::CAN_PACKET_SET_SERVO)
             << 8) |
            thrusters[i].vesc_id);
    }
    EXPECT_EQ(sent_frame_ids, expected_frame_ids);
}

TEST(CanModelTest, OnWriteDoesNotReplaceDisabledCommandsWithZero) {
    // FIXME: LispBMの実装が終わったらallowedコマンドの送信も検証する
    auto can = std::make_shared<Can>();
    auto sent_frames = std::vector<suchm::interface::CanFrame>{};

    EXPECT_CALL(*can, init(_)).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*can, send_frame(_))
        .Times(2)
        .WillRepeatedly(Invoke(
            [&sent_frames](
                const suchm::interface::CanFrame & frame) -> tl::expected<void, std::string> {
                sent_frames.push_back(frame);
                return {};
            }));

    const auto thrusters = make_thrusters(2);
    auto can_model = suchm::CanModel(can, thrusters, HARMONY_BMS_ID);
    ASSERT_TRUE(can_model.on_init());

    auto thruster_commands = make_enabled_commands(2);
    thruster_commands[0].esc_allowed.value = false;
    thruster_commands[1].servo_allowed.value = false;

    ASSERT_TRUE(can_model.on_write(
        succmd::power_distribution::Enabled{false}, thruster_commands,
        succmd::led_tape::Color{0, 0, 0}));
    ASSERT_TRUE(can_model.on_write(
        succmd::power_distribution::Enabled{false}, thruster_commands,
        succmd::led_tape::Color{0, 0, 0}));

    ASSERT_EQ(sent_frames.size(), 2u);
    EXPECT_EQ(sent_frames[0].id, thrusters[1].vesc_id);
    EXPECT_EQ(
        sent_frames[1].id, (static_cast<suchm::interface::CanFrame::Id>(
                                suchm::can::VescModel::PacketId::CAN_PACKET_SET_SERVO)
                            << 8) |
                               thrusters[0].vesc_id);
}

TEST_P(CanModelVariableThrusterCountTest, RoutesReceivedStatusToConfiguredThrusterName) {
    const auto thruster_count = GetParam();
    auto can = std::make_shared<Can>();
    const auto thrusters = make_thrusters(thruster_count);
    const auto vesc_id = thrusters.back().vesc_id;
    const auto frame = make_vesc_status_frame(
        vesc_id, suchm::can::VescModel::PacketStatus::ID,
        {std::byte{0x00}, std::byte{0x00}, std::byte{0x05}, std::byte{0x78}, std::byte{0x00},
         std::byte{0x7B}, std::byte{0x01}, std::byte{0xF4}});

    EXPECT_CALL(*can, recv_frames())
        .Times(1)
        .WillOnce(
            Return(tl::expected<std::vector<suchm::interface::CanFrame>, std::string>{{frame}}));

    auto can_model = suchm::CanModel(can, thrusters, HARMONY_BMS_ID);
    const auto result = can_model.on_read();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
    ASSERT_EQ(result.value().states.size(), 1u);
    EXPECT_TRUE(result.value().error_message.empty());
    const auto & [name, rpm] = std::get<0>(result.value().states[0]);
    EXPECT_EQ(name, "thruster" + std::to_string(thruster_count));
    EXPECT_DOUBLE_EQ(rpm.value, 200.0);
}

INSTANTIATE_TEST_SUITE_P(
    ThrusterCounts, CanModelVariableThrusterCountTest, testing::Values(1u, 3u, 4u, 6u));

}  // namespace sinsei_umiusi_control::test::hardware_model::can

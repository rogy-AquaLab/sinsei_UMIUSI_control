#include "mock/can.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>
#include <vector>

#include "gmock/gmock.h"
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
constexpr size_t PERIOD_LED_TAPE_PER_ACTUATORS = 2;

const auto ACTUATORS = std::vector<suchm::CanModel::ActuatorConfig>{
    {"thruster1", VESC_ID_1, suchm::CanModel::MotorType::BLDC, true},
    {"thruster2", VESC_ID_2, suchm::CanModel::MotorType::BLDC, true},
    {"thruster3", VESC_ID_3, suchm::CanModel::MotorType::BLDC, true},
    {"thruster4", VESC_ID_4, suchm::CanModel::MotorType::BLDC, true},
};

auto make_actuators(size_t count) -> std::vector<suchm::CanModel::ActuatorConfig> {
    auto actuators = std::vector<suchm::CanModel::ActuatorConfig>{};
    actuators.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        actuators.push_back(suchm::CanModel::ActuatorConfig{
            "thruster" + std::to_string(i + 1), static_cast<uint8_t>(i + 1),
            suchm::CanModel::MotorType::BLDC, true});
    }
    return actuators;
}

auto make_enabled_command(double duty_cycle = 0.25) -> suchm::CanModel::ActuatorCommand {
    return suchm::CanModel::ActuatorCommand{
        suchm::CanModel::MotorAllowed{true},
        suchm::CanModel::MotorDutyCycle{duty_cycle},
        suchm::CanModel::ServoAllowed{true},
        suchm::CanModel::ServoAngle{0.0},
    };
}

auto make_enabled_commands(size_t count) -> std::vector<suchm::CanModel::ActuatorCommand> {
    auto commands = std::vector<suchm::CanModel::ActuatorCommand>{};
    commands.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        commands.push_back(make_enabled_command());
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

    auto can_model = suchm::CanModel(can, ACTUATORS, PERIOD_LED_TAPE_PER_ACTUATORS);
    auto result = can_model.on_init();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
}

TEST(CanModelTest, CanModelOnDestroyTest) {
    auto can = std::make_shared<Can>();

    EXPECT_CALL(*can, close()).Times(1).WillOnce(Return(tl::expected<void, std::string>{}));

    auto can_model = suchm::CanModel(can, ACTUATORS, PERIOD_LED_TAPE_PER_ACTUATORS);
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

    auto can_model = suchm::CanModel(can, ACTUATORS, PERIOD_LED_TAPE_PER_ACTUATORS);
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

    auto can_model = suchm::CanModel(can, ACTUATORS, PERIOD_LED_TAPE_PER_ACTUATORS);
    const auto result = can_model.on_read();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
    const auto variant = result.value();
    ASSERT_EQ(variant.index(), 0u);
    const auto & [name, rpm] = std::get<0>(variant);
    EXPECT_EQ(name, "thruster1");
    EXPECT_DOUBLE_EQ(rpm.value, 200.0);
}

TEST(CanModelTest, CanModelOnReadDcMotorStatusDoesNotApplyBldcPoleConversionTest) {
    auto can = std::make_shared<Can>();
    const auto actuators = std::vector<suchm::CanModel::ActuatorConfig>{
        {"crawler_left", VESC_ID_1, suchm::CanModel::MotorType::DC, false}};
    const auto frame = make_vesc_status_frame(
        VESC_ID_1, suchm::can::PacketStatus::ID,
        {std::byte{0x00}, std::byte{0x00}, std::byte{0x05}, std::byte{0x78}, std::byte{0x00},
         std::byte{0x7B}, std::byte{0x01}, std::byte{0xF4}});

    EXPECT_CALL(*can, recv_frame())
        .Times(1)
        .WillOnce(
            Return(tl::expected<std::optional<suchm::interface::CanFrame>, std::string>{frame}));

    auto can_model = suchm::CanModel(can, actuators, PERIOD_LED_TAPE_PER_ACTUATORS);
    const auto result = can_model.on_read();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
    const auto & [name, rpm] = std::get<0>(result.value());
    EXPECT_EQ(name, "crawler_left");
    EXPECT_DOUBLE_EQ(rpm.value, 1400.0);
}

TEST(CanModelTest, CanModelOnReadServoOnlyActuatorIgnoresVescStatusTest) {
    auto can = std::make_shared<Can>();
    const auto actuators = std::vector<suchm::CanModel::ActuatorConfig>{
        {"servo_only", VESC_ID_1, suchm::CanModel::MotorType::None, true}};
    const auto frame = make_vesc_status_frame(VESC_ID_1, suchm::can::PacketStatus::ID);

    EXPECT_CALL(*can, recv_frame())
        .Times(1)
        .WillOnce(
            Return(tl::expected<std::optional<suchm::interface::CanFrame>, std::string>{frame}));

    auto can_model = suchm::CanModel(can, actuators, PERIOD_LED_TAPE_PER_ACTUATORS);
    const auto result = can_model.on_read();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
    EXPECT_EQ(result->index(), 7u);
    EXPECT_TRUE(std::holds_alternative<suchm::CanModel::IgnoredUpdate>(*result));
}

TEST(CanModelTest, CanModelOnReadUnsupportedPacketStatusReturnsErrorTest) {
    auto can = std::make_shared<Can>();

    const auto frame = make_vesc_status_frame(VESC_ID_1, suchm::can::PacketStatus2::ID);

    EXPECT_CALL(*can, recv_frame())
        .Times(1)
        .WillOnce(
            Return(tl::expected<std::optional<suchm::interface::CanFrame>, std::string>{frame}));

    auto can_model = suchm::CanModel(can, ACTUATORS, PERIOD_LED_TAPE_PER_ACTUATORS);
    const auto result = can_model.on_read();
    ASSERT_FALSE(result);
    EXPECT_EQ(
        result.error(),
        "Unsupported VESC packet status variant received (actuator 'thruster1' (VESC 1), "
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

    auto can_model = suchm::CanModel(can, ACTUATORS, PERIOD_LED_TAPE_PER_ACTUATORS);
    const auto result = can_model.on_read();
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), "Unhandled CAN frame: no registered model accepted frame id 2337");
}

TEST(CanModelTest, CanModelOnInitRejectsEmptyActuatorConfigurationTest) {
    auto can = std::make_shared<Can>();

    EXPECT_CALL(*can, init(_)).Times(0);

    auto can_model = suchm::CanModel(can, {}, PERIOD_LED_TAPE_PER_ACTUATORS);
    const auto result = can_model.on_init();
    ASSERT_FALSE(result);
    EXPECT_EQ(
        result.error(), "Invalid actuator configuration: At least one actuator must be configured");
}

TEST(CanModelTest, CanModelOnInitRejectsDuplicateActuatorNameTest) {
    auto can = std::make_shared<Can>();
    auto actuators = ACTUATORS;
    actuators[1].name = actuators[0].name;

    EXPECT_CALL(*can, init(_)).Times(0);

    auto can_model = suchm::CanModel(can, actuators, PERIOD_LED_TAPE_PER_ACTUATORS);
    const auto result = can_model.on_init();
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), "Invalid actuator configuration: Duplicate actuator name: thruster1");
}

TEST(CanModelTest, CanModelOnInitRejectsDuplicateVescIdTest) {
    auto can = std::make_shared<Can>();
    auto actuators = ACTUATORS;
    actuators[1].vesc_id = actuators[0].vesc_id;

    EXPECT_CALL(*can, init(_)).Times(0);

    auto can_model = suchm::CanModel(can, actuators, PERIOD_LED_TAPE_PER_ACTUATORS);
    const auto result = can_model.on_init();
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), "Invalid actuator configuration: Duplicate VESC ID: 1");
}

TEST(CanModelTest, CanModelOnInitRejectsActuatorWithoutMotorOrServoTest) {
    auto can = std::make_shared<Can>();
    const auto actuators = std::vector<suchm::CanModel::ActuatorConfig>{
        {"empty", VESC_ID_1, suchm::CanModel::MotorType::None, false}};

    EXPECT_CALL(*can, init(_)).Times(0);

    auto can_model = suchm::CanModel(can, actuators, PERIOD_LED_TAPE_PER_ACTUATORS);
    const auto result = can_model.on_init();
    ASSERT_FALSE(result);
    EXPECT_EQ(
        result.error(),
        "Invalid actuator configuration: Actuator must have a motor or servo: empty");
}

TEST(CanModelTest, CanModelOnInitRejectsInvalidLedTapePeriodTest) {
    auto can = std::make_shared<Can>();

    EXPECT_CALL(*can, init(_)).Times(0);

    auto can_model = suchm::CanModel(can, ACTUATORS, 1);
    const auto result = can_model.on_init();
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error(), "period_led_tape_per_actuators must be greater than 1");
}

// FIXME: MainPowerModelが未実装のため、main_powerの状態変化を伴うon_writeは未テスト
TEST(CanModelTest, OnWriteUsesCommandIndexForConfiguredActuatorOrder) {
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

    constexpr size_t ACTUATOR_COUNT = 3;
    constexpr size_t PERIOD_WITHOUT_LED_IN_FIRST_CYCLE = 10;
    auto can_model =
        suchm::CanModel(can, make_actuators(ACTUATOR_COUNT), PERIOD_WITHOUT_LED_IN_FIRST_CYCLE);
    ASSERT_TRUE(can_model.on_init());

    // コマンドのインデックスがアクチュエータ設定のインデックスに対応することを確認する
    const auto actuator_commands = std::vector<suchm::CanModel::ActuatorCommand>{
        make_enabled_command(0.25),
        make_enabled_command(0.50),
        make_enabled_command(0.75),
    };

    // 最初の2回はモーター許可のno-op枠で、続く3回が各アクチュエータのDutyコマンドになる
    for (size_t i = 0; i < 5; ++i) {
        can_model.on_write(
            succmd::main_power::Enabled{false}, actuator_commands,
            succmd::led_tape::Color{0, 0, 0});
    }

    ASSERT_EQ(sent_frames.size(), ACTUATOR_COUNT);
    EXPECT_EQ(sent_frames[0].id, VESC_ID_1);
    EXPECT_EQ(sent_frames[1].id, VESC_ID_2);
    EXPECT_EQ(sent_frames[2].id, VESC_ID_3);
    EXPECT_EQ(sinsei_umiusi_control::util::to_int32_be(sent_frames[0].data).value(), 25000);
    EXPECT_EQ(sinsei_umiusi_control::util::to_int32_be(sent_frames[1].data).value(), 50000);
    EXPECT_EQ(sinsei_umiusi_control::util::to_int32_be(sent_frames[2].data).value(), 75000);
}

TEST(CanModelTest, OnWriteRejectsMismatchedActuatorCommandCount) {
    auto can = std::make_shared<Can>();

    EXPECT_CALL(*can, init(_)).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*can, send_frame(_)).Times(0);

    constexpr size_t ACTUATOR_COUNT = 3;
    auto can_model =
        suchm::CanModel(can, make_actuators(ACTUATOR_COUNT), PERIOD_LED_TAPE_PER_ACTUATORS);
    ASSERT_TRUE(can_model.on_init());

    auto actuator_commands = make_enabled_commands(ACTUATOR_COUNT);
    actuator_commands.pop_back();
    const auto result = can_model.on_write(
        succmd::main_power::Enabled{false}, actuator_commands, succmd::led_tape::Color{0, 0, 0});

    ASSERT_FALSE(result);
    EXPECT_EQ(
        result.error(), "Actuator command count does not match configuration: expected 3, got 2");
}

TEST(CanModelTest, OnWriteSendsZeroDutyWhenMotorIsNotAllowedTest) {
    auto can = std::make_shared<Can>();
    const auto actuators = std::vector<suchm::CanModel::ActuatorConfig>{
        {"crawler_left", VESC_ID_1, suchm::CanModel::MotorType::DC, false}};
    auto sent_frame = suchm::interface::CanFrame{};

    EXPECT_CALL(*can, init(_)).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*can, send_frame(_))
        .Times(1)
        .WillOnce(Invoke(
            [&sent_frame](
                const suchm::interface::CanFrame & frame) -> tl::expected<void, std::string> {
                sent_frame = frame;
                return {};
            }));

    auto can_model = suchm::CanModel(can, actuators, PERIOD_LED_TAPE_PER_ACTUATORS);
    ASSERT_TRUE(can_model.on_init());
    auto command = make_enabled_command(0.75);
    command.motor_allowed.value = false;

    const auto result = can_model.on_write(
        succmd::main_power::Enabled{false}, {command}, succmd::led_tape::Color{0, 0, 0});

    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
    EXPECT_EQ(sent_frame.id, VESC_ID_1);
    EXPECT_EQ(sinsei_umiusi_control::util::to_int32_be(sent_frame.data).value(), 0);
}

TEST(CanModelTest, OnWriteSupportsMixedMotorAndServoCapabilitiesTest) {
    auto can = std::make_shared<Can>();
    auto sent_frames = std::vector<suchm::interface::CanFrame>{};
    const auto actuators = std::vector<suchm::CanModel::ActuatorConfig>{
        {"thruster", VESC_ID_1, suchm::CanModel::MotorType::BLDC, true},
        {"crawler", VESC_ID_2, suchm::CanModel::MotorType::DC, false},
        {"servo_only", VESC_ID_3, suchm::CanModel::MotorType::None, true},
    };

    EXPECT_CALL(*can, init(_)).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*can, send_frame(_))
        .Times(4)
        .WillRepeatedly(Invoke(
            [&sent_frames](
                const suchm::interface::CanFrame & frame) -> tl::expected<void, std::string> {
                sent_frames.push_back(frame);
                return {};
            }));

    constexpr size_t PERIOD_WITHOUT_LED_IN_FIRST_CYCLE = 10;
    auto can_model = suchm::CanModel(can, actuators, PERIOD_WITHOUT_LED_IN_FIRST_CYCLE);
    ASSERT_TRUE(can_model.on_init());
    const auto commands = std::vector<suchm::CanModel::ActuatorCommand>{
        make_enabled_command(0.25), make_enabled_command(0.50), make_enabled_command(0.75)};

    constexpr size_t COMMAND_TYPES_PER_ACTUATOR = 4;
    for (size_t i = 0; i < actuators.size() * COMMAND_TYPES_PER_ACTUATOR; ++i) {
        can_model.on_write(
            succmd::main_power::Enabled{false}, commands, succmd::led_tape::Color{0, 0, 0});
    }

    ASSERT_EQ(sent_frames.size(), 4u);
    EXPECT_EQ(sent_frames[0].id, VESC_ID_1);
    EXPECT_EQ(sent_frames[1].id, VESC_ID_2);
    EXPECT_EQ(
        sent_frames[2].id, (static_cast<suchm::interface::CanFrame::Id>(
                                suchm::can::VescSimpleCommandID::CAN_PACKET_SET_SERVO)
                            << 8) |
                               VESC_ID_1);
    EXPECT_EQ(
        sent_frames[3].id, (static_cast<suchm::interface::CanFrame::Id>(
                                suchm::can::VescSimpleCommandID::CAN_PACKET_SET_SERVO)
                            << 8) |
                               VESC_ID_3);
}

class CanModelVariableActuatorCountTest : public testing::TestWithParam<size_t> {};

TEST_P(CanModelVariableActuatorCountTest, WritesEachDynamicActuatorInRoundRobinOrder) {
    const auto actuator_count = GetParam();
    auto can = std::make_shared<Can>();
    auto sent_frame_ids = std::vector<suchm::interface::CanFrame::Id>{};

    EXPECT_CALL(*can, send_frame(_))
        .Times(2 * actuator_count)
        .WillRepeatedly(Invoke(
            [&sent_frame_ids](
                const suchm::interface::CanFrame & frame) -> tl::expected<void, std::string> {
                sent_frame_ids.push_back(frame.id);
                return {};
            }));

    constexpr size_t PERIOD_WITHOUT_LED_IN_FIRST_CYCLE = 10;
    auto can_model =
        suchm::CanModel(can, make_actuators(actuator_count), PERIOD_WITHOUT_LED_IN_FIRST_CYCLE);
    EXPECT_CALL(*can, init(_)).WillOnce(Return(tl::expected<void, std::string>{}));
    ASSERT_TRUE(can_model.on_init());

    auto actuator_commands = make_enabled_commands(actuator_count);

    constexpr size_t COMMAND_TYPES_PER_ACTUATOR = 4;
    // allowedと存在しない機能の枠は正常なno-op、Duty比とサーボ角度の枠だけCAN送信する。
    for (size_t i = 0; i < actuator_count * COMMAND_TYPES_PER_ACTUATOR; ++i) {
        const auto result = can_model.on_write(
            succmd::main_power::Enabled{false}, actuator_commands,
            succmd::led_tape::Color{0, 0, 0});
        EXPECT_TRUE(result) << std::string("Error: ") + result.error();
    }

    auto expected_frame_ids = std::vector<suchm::interface::CanFrame::Id>{};
    expected_frame_ids.reserve(2 * actuator_count);
    for (size_t i = 0; i < actuator_count; ++i) {
        expected_frame_ids.push_back(static_cast<uint8_t>(i + 1));
    }
    for (size_t i = 0; i < actuator_count; ++i) {
        expected_frame_ids.push_back(
            (static_cast<suchm::interface::CanFrame::Id>(
                 suchm::can::VescSimpleCommandID::CAN_PACKET_SET_SERVO)
             << 8) |
            static_cast<uint8_t>(i + 1));
    }
    EXPECT_EQ(sent_frame_ids, expected_frame_ids);
}

TEST_P(CanModelVariableActuatorCountTest, RoutesReceivedStatusToConfiguredActuatorName) {
    const auto actuator_count = GetParam();
    auto can = std::make_shared<Can>();
    const auto vesc_id = static_cast<uint8_t>(actuator_count);
    const auto frame = make_vesc_status_frame(
        vesc_id, suchm::can::PacketStatus::ID,
        {std::byte{0x00}, std::byte{0x00}, std::byte{0x05}, std::byte{0x78}, std::byte{0x00},
         std::byte{0x7B}, std::byte{0x01}, std::byte{0xF4}});

    EXPECT_CALL(*can, recv_frame())
        .Times(1)
        .WillOnce(
            Return(tl::expected<std::optional<suchm::interface::CanFrame>, std::string>{frame}));

    auto can_model =
        suchm::CanModel(can, make_actuators(actuator_count), PERIOD_LED_TAPE_PER_ACTUATORS);
    const auto result = can_model.on_read();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
    const auto & [name, rpm] = std::get<0>(result.value());
    EXPECT_EQ(name, "thruster" + std::to_string(actuator_count));
    EXPECT_DOUBLE_EQ(rpm.value, 200.0);
}

INSTANTIATE_TEST_SUITE_P(
    ActuatorCounts, CanModelVariableActuatorCountTest, testing::Values(1u, 3u, 4u, 6u));

}  // namespace sinsei_umiusi_control::test::hardware_model::can

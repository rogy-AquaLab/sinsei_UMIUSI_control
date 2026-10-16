#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <variant>

#include "sinsei_umiusi_control/hardware_model/can/stepping_motor_driver_model.hpp"
#include "sinsei_umiusi_control/hardware_model/interface/can.hpp"

namespace suchm = sinsei_umiusi_control::hardware_model;

namespace sinsei_umiusi_control::test::hardware_model::can::stepping_motor_driver {

namespace {

using Model = suchm::can::SteppingMotorDriverModel;

auto make_frame(
    suchm::interface::CanFrame::Id id, suchm::interface::CanFrame::Data data, uint8_t length = 8,
    bool is_extended = false) -> suchm::interface::CanFrame {
    return {id, length, data, is_extended};
}

}  // namespace

TEST(SteppingMotorDriverModelTest, DecodeStatusTest) {
    // can_protocol.mdの受信例
    const auto frame = make_frame(
        0x318, {std::byte{0x13}, std::byte{0x88}, std::byte{0xFF}, std::byte{0xFF}, std::byte{0x05},
                std::byte{0x16}, std::byte{0x04}, std::byte{0x00}});

    const auto result = Model::decode(frame);
    ASSERT_TRUE(result) << result.error();
    ASSERT_TRUE(result.value());
    const auto * status = std::get_if<Model::PacketStatus>(&result.value().value());
    ASSERT_NE(status, nullptr);

    EXPECT_DOUBLE_EQ(status->axes[0].position, 0.5);
    EXPECT_EQ(status->axes[0].state, static_cast<uint8_t>(Model::AxisState::Ready));
    EXPECT_EQ(status->axes[0].fault_reason, 0);
    EXPECT_EQ(status->axes[0].flags, 0x04);

    EXPECT_TRUE(std::isnan(status->axes[1].position));
    EXPECT_EQ(status->axes[1].state, static_cast<uint8_t>(Model::AxisState::Fault));
    EXPECT_EQ(status->axes[1].fault_reason, 1);
    EXPECT_EQ(status->axes[1].flags, 0x00);
}

TEST(SteppingMotorDriverModelTest, DecodeMeasuredTravelTest) {
    // 180844 = 0x0002C26C
    const auto frame = make_frame(
        0x319,
        {std::byte{0x00}, std::byte{0x00}, std::byte{0x02}, std::byte{0xC2}, std::byte{0x6C},
         std::byte{0x00}, std::byte{0x00}, std::byte{0x00}},
        5);

    const auto result = Model::decode(frame);
    ASSERT_TRUE(result) << result.error();
    ASSERT_TRUE(result.value());
    const auto * travel = std::get_if<Model::PacketMeasuredTravel>(&result.value().value());
    ASSERT_NE(travel, nullptr);
    EXPECT_EQ(travel->axis, 0);
    EXPECT_EQ(travel->steps, 180844);
}

TEST(SteppingMotorDriverModelTest, DecodeShortStatusReturnsErrorTest) {
    const auto frame = make_frame(0x318, {}, 7);

    const auto result = Model::decode(frame);
    ASSERT_FALSE(result);
    EXPECT_EQ(
        result.error(), "Stepping motor driver frame 792 is too short: expected 8 bytes, got 7");
}

TEST(SteppingMotorDriverModelTest, DecodeCommandFrameReturnsMonostateTest) {
    const auto frame = make_frame(0x311, {std::byte{0x01}, std::byte{0x27}, std::byte{0x10}}, 3);

    const auto result = Model::decode(frame);
    ASSERT_TRUE(result) << result.error();
    ASSERT_TRUE(result.value());
    EXPECT_TRUE(std::holds_alternative<std::monostate>(result.value().value()));
}

TEST(SteppingMotorDriverModelTest, DecodeIgnoresOtherFramesTest) {
    // 拡張IDのフレーム（VESCなど）
    const auto extended_result = Model::decode(make_frame(0x318, {}, 8, true));
    ASSERT_TRUE(extended_result) << extended_result.error();
    EXPECT_FALSE(extended_result.value());

    // 範囲外の標準ID
    const auto other_result = Model::decode(make_frame(0x320, {}));
    ASSERT_TRUE(other_result) << other_result.error();
    EXPECT_FALSE(other_result.value());
}

TEST(SteppingMotorDriverModelTest, MakeTargetFrameTest) {
    // can_protocol.mdの例: M2だけ100.00 %へ
    const auto result = Model::make_target_frame(1, 1.0);
    ASSERT_TRUE(result) << result.error();
    const auto & frame = result.value();
    EXPECT_EQ(frame.id, 0x311u);
    EXPECT_FALSE(frame.is_extended);
    ASSERT_EQ(frame.len, 3);
    EXPECT_EQ(frame.data[0], std::byte{0x01});
    EXPECT_EQ(frame.data[1], std::byte{0x27});
    EXPECT_EQ(frame.data[2], std::byte{0x10});
}

TEST(SteppingMotorDriverModelTest, MakeTargetFrameRoundsToHundredthPercentTest) {
    const auto result = Model::make_target_frame(0, 0.12346);
    ASSERT_TRUE(result) << result.error();
    // 0.12346 * 10000 = 1234.6 -> 1235 = 0x04D3
    EXPECT_EQ(result.value().data[1], std::byte{0x04});
    EXPECT_EQ(result.value().data[2], std::byte{0xD3});
}

TEST(SteppingMotorDriverModelTest, MakeTargetFrameRejectsInvalidArgumentsTest) {
    EXPECT_FALSE(Model::make_target_frame(2, 0.5));
    EXPECT_FALSE(Model::make_target_frame(0, -0.1));
    EXPECT_FALSE(Model::make_target_frame(0, 1.1));
    EXPECT_FALSE(Model::make_target_frame(0, std::nan("")));
}

TEST(SteppingMotorDriverModelTest, MakeCalibrateFrameTest) {
    const auto result = Model::make_calibrate_frame(0x01);
    ASSERT_TRUE(result) << result.error();
    EXPECT_EQ(result.value().id, 0x312u);
    EXPECT_FALSE(result.value().is_extended);
    ASSERT_EQ(result.value().len, 1);
    EXPECT_EQ(result.value().data[0], std::byte{0x01});

    EXPECT_FALSE(Model::make_calibrate_frame(0x00));
    EXPECT_FALSE(Model::make_calibrate_frame(0x04));
}

}  // namespace sinsei_umiusi_control::test::hardware_model::can::stepping_motor_driver

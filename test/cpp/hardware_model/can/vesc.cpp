#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rcpputils/tl_expected/expected.hpp>

#include "gmock/gmock.h"
#include "sinsei_umiusi_control/hardware_model/can/vesc_model.hpp"
#include "sinsei_umiusi_control/hardware_model/interface/can.hpp"
#include "sinsei_umiusi_control/util/byte.hpp"

namespace sucutil = sinsei_umiusi_control::util;
namespace suchm = sinsei_umiusi_control::hardware_model;

using testing::Return;

namespace sinsei_umiusi_control::test::hardware_model::can::vesc {

namespace {

constexpr uint8_t DUMMY_ID = 0x01;

}  // namespace

TEST(VescModelTest, VescModelMakeDutyFrameValidTest) {
    auto vesc_model = suchm::can::VescModel{DUMMY_ID};
    auto result = vesc_model.make_duty_frame(0.5);
    ASSERT_TRUE(result);
    const auto frame = result.value();
    EXPECT_EQ(
        frame.id, (static_cast<suchm::interface::CanFrame::Id>(
                       suchm::can::VescSimpleCommandID::CAN_PACKET_SET_DUTY) &
                   0xFF) << 8 |
                      DUMMY_ID);
    EXPECT_EQ(frame.len, 4);
    EXPECT_EQ(sucutil::to_int32_be(frame.data), 50000);  // 0.5 * 100000
}

TEST(VescModelTest, VescModelMakeDutyFrameInvalidTest) {
    auto vesc_model = suchm::can::VescModel{DUMMY_ID};
    auto result = vesc_model.make_duty_frame(1.5);
    ASSERT_FALSE(result);
}

TEST(VescModelTest, VescModelMakeRpmFrameTest) {
    auto vesc_model = suchm::can::VescModel{DUMMY_ID};
    auto result = vesc_model.make_rpm_frame(100);
    ASSERT_TRUE(result);
    const auto frame = result.value();
    EXPECT_EQ(
        frame.id, (static_cast<suchm::interface::CanFrame::Id>(
                       suchm::can::VescSimpleCommandID::CAN_PACKET_SET_RPM) &
                   0xFF) << 8 |
                      DUMMY_ID);
    EXPECT_EQ(frame.len, 4);
    EXPECT_EQ(sucutil::to_int32_be(frame.data), 100);  // 100 * 1
}

TEST(VescModelTest, VescModelMakeServoAngleFrameValidTest) {
    auto vesc_model = suchm::can::VescModel{DUMMY_ID};
    auto result = vesc_model.make_servo_angle_frame(90.0);
    ASSERT_TRUE(result);
    const auto frame = result.value();
    EXPECT_EQ(
        frame.id, (static_cast<suchm::interface::CanFrame::Id>(
                       suchm::can::VescSimpleCommandID::CAN_PACKET_SET_SERVO) &
                   0xFF) << 8 |
                      DUMMY_ID);
    EXPECT_EQ(frame.len, 4);
    EXPECT_EQ(sucutil::to_int32_be(frame.data), 10000);  // (90 + 90) / 180 * 10000
}

TEST(VescModelTest, VescModelMakeServoAngleFrameInvalidTest) {
    auto vesc_model = suchm::can::VescModel{DUMMY_ID};
    auto result = vesc_model.make_servo_angle_frame(200.0);
    ASSERT_FALSE(result);
}

TEST(VescModelTest, VescModelGetRpmValidTest) {
    auto vesc_model = suchm::can::VescModel{DUMMY_ID};
    const auto frame = suchm::interface::CanFrame{
        (static_cast<suchm::interface::CanFrame::Id>(
             suchm::can::VescStatusCommandID::CAN_PACKET_STATUS) &
         0xFF) << 8 |
            DUMMY_ID,
        8,
        sucutil::to_bytes_be(static_cast<int32_t>(700)),  // 100 RPM * 7
        true};
    auto result = vesc_model.get_rpm(frame);
    ASSERT_TRUE(result) << "Error: " << result.error();
    ASSERT_TRUE(result.value());
    EXPECT_EQ(result.value()->value, 100);  // NOLINT(bugprone-unchecked-optional-access)
}

TEST(VescModelTest, VescModelGetRpmUnmatchedIdTest) {
    auto vesc_model = suchm::can::VescModel{DUMMY_ID};
    const auto frame = suchm::interface::CanFrame{
        (static_cast<suchm::interface::CanFrame::Id>(
             suchm::can::VescStatusCommandID::CAN_PACKET_STATUS) &
         0xFF) << 8 |
            (DUMMY_ID + 1),  // Different ID
        8,
        {},
        true};
    auto result = vesc_model.get_rpm(frame);
    ASSERT_TRUE(result);           // Should not fail,
    ASSERT_FALSE(result.value());  // but return no value
}

TEST(VescModelTest, VescModelGetRpmInvalidFrameLengthTest) {
    auto vesc_model = suchm::can::VescModel{DUMMY_ID};
    const auto frame = suchm::interface::CanFrame{
        (static_cast<suchm::interface::CanFrame::Id>(
             suchm::can::VescStatusCommandID::CAN_PACKET_STATUS) &
         0xFF) << 8 |
            DUMMY_ID,
        4,  // Invalid length
        {},
        true};
    auto result = vesc_model.get_rpm(frame);
    ASSERT_FALSE(result);
}

TEST(VescModelTest, VescModelGetRpmInvalidCmdIdTest) {
    auto vesc_model = suchm::can::VescModel{DUMMY_ID};
    const auto frame = suchm::interface::CanFrame{
        0xFF << 8 | DUMMY_ID,  // Invalid command ID
        8,
        {},
        true};
    auto result = vesc_model.get_rpm(frame);
    ASSERT_FALSE(result);
}

TEST(VescModelTest, VescModelUnmatchedCommandIdTest) {
    auto vesc_model = suchm::can::VescModel{DUMMY_ID};
    const auto frame = suchm::interface::CanFrame{
        (static_cast<suchm::interface::CanFrame::Id>(
             suchm::can::VescStatusCommandID::CAN_PACKET_STATUS_2) &
         0xFF) << 8 |
            DUMMY_ID,
        8,
        {},
        true};
    auto result = vesc_model.get_rpm(frame);
    ASSERT_TRUE(result);           // Should not fail,
    ASSERT_FALSE(result.value());  // but return no value
}

TEST(VescModelTest, VescModelGetWaterLeakedValidTest) {
    auto vesc_model = suchm::can::VescModel{DUMMY_ID};
    const auto frame1 = suchm::interface::CanFrame{
        (static_cast<suchm::interface::CanFrame::Id>(
             suchm::can::VescStatusCommandID::CAN_PACKET_STATUS_6) &
         0xFF) << 8 |
            DUMMY_ID,
        8,
        sucutil::to_bytes_be(static_cast<int16_t>(1000)),  // 1.0 V * 1000 (not leaked)
        true};
    auto result1 = vesc_model.get_water_leaked(frame1);
    ASSERT_TRUE(result1) << "Error: " << result1.error();
    ASSERT_TRUE(result1.value());
    EXPECT_EQ(result1.value()->value, false);  // NOLINT(bugprone-unchecked-optional-access)
    const auto frame2 = suchm::interface::CanFrame{
        (static_cast<suchm::interface::CanFrame::Id>(
             suchm::can::VescStatusCommandID::CAN_PACKET_STATUS_6) &
         0xFF) << 8 |
            DUMMY_ID,
        8,
        sucutil::to_bytes_be(static_cast<int16_t>(3000)),  // 3.0 V * 1000 (leaked)
        true};
    auto result2 = vesc_model.get_water_leaked(frame2);
    ASSERT_TRUE(result2) << "Error: " << result2.error();
    ASSERT_TRUE(result2.value());
    EXPECT_EQ(result2.value()->value, true);  // NOLINT(bugprone-unchecked-optional-access)
}

TEST(VescModelTest, VescModelGetWaterLeakedUnmatchedIdTest) {
    auto vesc_model = suchm::can::VescModel{DUMMY_ID};
    const auto frame = suchm::interface::CanFrame{
        (static_cast<suchm::interface::CanFrame::Id>(
             suchm::can::VescStatusCommandID::CAN_PACKET_STATUS_6) &
         0xFF) << 8 |
            (DUMMY_ID + 1),  // Different ID
        8,
        {},
        true};
    auto result = vesc_model.get_water_leaked(frame);
    ASSERT_TRUE(result);           // Should not fail,
    ASSERT_FALSE(result.value());  // but return no value
}

TEST(VescModelTest, VescModelGetWaterLeakedInvalidFrameLengthTest) {
    auto vesc_model = suchm::can::VescModel{DUMMY_ID};
    const auto frame = suchm::interface::CanFrame{
        (static_cast<suchm::interface::CanFrame::Id>(
             suchm::can::VescStatusCommandID::CAN_PACKET_STATUS_6) &
         0xFF) << 8 |
            DUMMY_ID,
        4,  // Invalid length
        {},
        true};
    auto result = vesc_model.get_water_leaked(frame);
    ASSERT_FALSE(result);
}

TEST(VescModelTest, VescModelGetWaterLeakedInvalidCmdIdTest) {
    auto vesc_model = suchm::can::VescModel{DUMMY_ID};
    const auto frame = suchm::interface::CanFrame{
        0xFF << 8 | DUMMY_ID,  // Invalid command ID
        8,
        {},
        true};
    auto result = vesc_model.get_water_leaked(frame);
    ASSERT_FALSE(result);
}

TEST(VescModelTest, VescModelGetWaterLeakedUnmatchedCommandIdTest) {
    auto vesc_model = suchm::can::VescModel{DUMMY_ID};
    const auto frame = suchm::interface::CanFrame{
        (static_cast<suchm::interface::CanFrame::Id>(
             suchm::can::VescStatusCommandID::CAN_PACKET_STATUS_5) &
         0xFF) << 8 |
            DUMMY_ID,
        8,
        {},
        true};
    auto result = vesc_model.get_water_leaked(frame);
    ASSERT_TRUE(result);           // Should not fail,
    ASSERT_FALSE(result.value());  // but return no value
}

}  // namespace sinsei_umiusi_control::test::hardware_model::can::vesc

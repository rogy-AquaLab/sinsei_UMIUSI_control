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

}  // namespace sinsei_umiusi_control::test::hardware_model::can::vesc

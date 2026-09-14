#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <boost/math/constants/constants.hpp>
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
    constexpr auto HALF_PI = boost::math::constants::pi<double>() / 2.0;
    auto result = vesc_model.make_servo_angle_frame(HALF_PI);
    ASSERT_TRUE(result);
    const auto frame = result.value();
    EXPECT_EQ(
        frame.id, (static_cast<suchm::interface::CanFrame::Id>(
                       suchm::can::VescSimpleCommandID::CAN_PACKET_SET_SERVO) &
                   0xFF) << 8 |
                      DUMMY_ID);
    EXPECT_EQ(frame.len, 4);
    EXPECT_EQ(sucutil::to_int32_be(frame.data), 10000);  // pi/2 rad -> 1.0 -> 10000
}

TEST(VescModelTest, VescModelConvertsServoAngleFromRadiansAtSendBoundaryTest) {
    auto vesc_model = suchm::can::VescModel{DUMMY_ID};
    constexpr auto QUARTER_PI = boost::math::constants::pi<double>() / 4.0;

    const auto center_result = vesc_model.make_servo_angle_frame(0.0);
    ASSERT_TRUE(center_result);
    EXPECT_EQ(sucutil::to_int32_be(center_result->data), 5000);

    const auto negative_result = vesc_model.make_servo_angle_frame(-QUARTER_PI);
    ASSERT_TRUE(negative_result);
    EXPECT_EQ(sucutil::to_int32_be(negative_result->data), 2500);
}

TEST(VescModelTest, VescModelMakeServoAngleFrameInvalidTest) {
    auto vesc_model = suchm::can::VescModel{DUMMY_ID};
    constexpr auto PI = boost::math::constants::pi<double>();
    auto result = vesc_model.make_servo_angle_frame(PI);
    ASSERT_FALSE(result);
}

}  // namespace sinsei_umiusi_control::test::hardware_model::can::vesc

#include <gtest/gtest.h>

#include <array>
#include <boost/math/constants/constants.hpp>
#include <cstddef>
#include <cstdint>

#include "sinsei_umiusi_control/hardware_model/can/vesc_model.hpp"
#include "sinsei_umiusi_control/hardware_model/interface/can.hpp"
#include "sinsei_umiusi_control/util/byte.hpp"

namespace sucutil = sinsei_umiusi_control::util;
namespace suchm = sinsei_umiusi_control::hardware_model;

namespace sinsei_umiusi_control::test::hardware_model::can::vesc {

namespace {

using VescModel = suchm::can::VescModel;

constexpr uint8_t DUMMY_ID = 0x01;

auto make_frame(
    VescModel::PacketId packet_id, suchm::interface::CanFrame::Data data = {}, uint8_t length = 8,
    uint8_t node_id = DUMMY_ID, bool is_extended = true) -> suchm::interface::CanFrame {
    return {
        (static_cast<uint32_t>(packet_id) << 8) | node_id,
        length,
        data,
        is_extended,
    };
}

auto two_int32s(int32_t first, int32_t second) -> suchm::interface::CanFrame::Data {
    const auto first_bytes = sucutil::to_bytes_be(first);
    const auto second_bytes = sucutil::to_bytes_be(second);
    return {
        first_bytes[0],  first_bytes[1],  first_bytes[2],  first_bytes[3],
        second_bytes[0], second_bytes[1], second_bytes[2], second_bytes[3],
    };
}

auto four_int16s(int16_t first, int16_t second, int16_t third, int16_t fourth)
    -> suchm::interface::CanFrame::Data {
    const auto first_bytes = sucutil::to_bytes_be(first);
    const auto second_bytes = sucutil::to_bytes_be(second);
    const auto third_bytes = sucutil::to_bytes_be(third);
    const auto fourth_bytes = sucutil::to_bytes_be(fourth);
    return {
        first_bytes[0], first_bytes[1], second_bytes[0], second_bytes[1],
        third_bytes[0], third_bytes[1], fourth_bytes[0], fourth_bytes[1],
    };
}

auto int32_and_int16(int32_t first, int16_t second) -> suchm::interface::CanFrame::Data {
    const auto first_bytes = sucutil::to_bytes_be(first);
    const auto second_bytes = sucutil::to_bytes_be(second);
    return {
        first_bytes[0],  first_bytes[1],  first_bytes[2], first_bytes[3],
        second_bytes[0], second_bytes[1], std::byte{0},   std::byte{0},
    };
}

}  // namespace

TEST(VescModelTest, VescModelExposesConfiguredIdTest) {
    const auto model = VescModel{DUMMY_ID};
    EXPECT_EQ(model.get_id(), DUMMY_ID);
}

TEST(VescModelTest, VescModelMakeDutyFrameValidTest) {
    const auto model = VescModel{DUMMY_ID};
    const auto result = model.make_duty_frame(0.5);
    ASSERT_TRUE(result);
    const auto frame = result.value();
    EXPECT_EQ(
        frame.id,
        (static_cast<suchm::interface::CanFrame::Id>(VescModel::PacketId::CAN_PACKET_SET_DUTY)
         << 8) |
            DUMMY_ID);
    EXPECT_EQ(frame.len, 4);
    EXPECT_EQ(sucutil::to_int32_be(frame.data), 50000);
}

TEST(VescModelTest, VescModelMakeDutyFrameInvalidTest) {
    const auto model = VescModel{DUMMY_ID};
    EXPECT_FALSE(model.make_duty_frame(1.5));
}

TEST(VescModelTest, VescModelMakeRpmFrameTest) {
    const auto model = VescModel{DUMMY_ID};
    const auto result = model.make_rpm_frame(100);
    ASSERT_TRUE(result);
    const auto frame = result.value();
    EXPECT_EQ(
        frame.id,
        (static_cast<suchm::interface::CanFrame::Id>(VescModel::PacketId::CAN_PACKET_SET_RPM)
         << 8) |
            DUMMY_ID);
    EXPECT_EQ(frame.len, 4);
    EXPECT_EQ(sucutil::to_int32_be(frame.data), 100);
}

TEST(VescModelTest, VescModelMakeServoAngleFrameValidTest) {
    const auto model = VescModel{DUMMY_ID};
    constexpr auto HALF_PI = boost::math::constants::pi<double>() / 2.0;
    const auto result = model.make_servo_angle_frame(HALF_PI);
    ASSERT_TRUE(result);
    const auto frame = result.value();
    EXPECT_EQ(
        frame.id,
        (static_cast<suchm::interface::CanFrame::Id>(VescModel::PacketId::CAN_PACKET_SET_SERVO)
         << 8) |
            DUMMY_ID);
    EXPECT_EQ(frame.len, 4);
    EXPECT_EQ(sucutil::to_int32_be(frame.data), 10000);
}

TEST(VescModelTest, VescModelConvertsServoAngleFromRadiansAtSendBoundaryTest) {
    const auto model = VescModel{DUMMY_ID};
    constexpr auto QUARTER_PI = boost::math::constants::pi<double>() / 4.0;

    const auto center_result = model.make_servo_angle_frame(0.0);
    ASSERT_TRUE(center_result);
    EXPECT_EQ(sucutil::to_int32_be(center_result->data), 5000);

    const auto negative_result = model.make_servo_angle_frame(-QUARTER_PI);
    ASSERT_TRUE(negative_result);
    EXPECT_EQ(sucutil::to_int32_be(negative_result->data), 2500);
}

TEST(VescModelTest, VescModelMakeServoAngleFrameInvalidTest) {
    const auto model = VescModel{DUMMY_ID};
    constexpr auto PI = boost::math::constants::pi<double>();
    EXPECT_FALSE(model.make_servo_angle_frame(PI));
}

TEST(VescModelTest, VescModelIgnoresFramesFromOtherNodesAndStandardFramesTest) {
    const auto model = VescModel{DUMMY_ID};

    const auto other_node_result =
        model.decode(make_frame(VescModel::PacketStatus::ID, {}, 8, 0x02));
    ASSERT_TRUE(other_node_result);
    EXPECT_FALSE(other_node_result.value());

    const auto standard_frame_result =
        model.decode(make_frame(VescModel::PacketStatus::ID, {}, 8, DUMMY_ID, false));
    ASSERT_TRUE(standard_frame_result);
    EXPECT_FALSE(standard_frame_result.value());
}

TEST(VescModelTest, VescModelRejectsInvalidLengthAndUnknownPacketTest) {
    const auto model = VescModel{DUMMY_ID};

    const auto invalid_length_result = model.decode(make_frame(VescModel::PacketStatus::ID, {}, 4));
    ASSERT_FALSE(invalid_length_result);
    EXPECT_EQ(
        invalid_length_result.error(),
        "Received CAN frame with invalid length (expected: 8, received: 4)");

    const auto unknown_packet_result =
        model.decode(make_frame(static_cast<VescModel::PacketId>(0x7F)));
    ASSERT_FALSE(unknown_packet_result);
    EXPECT_EQ(unknown_packet_result.error(), "Received CAN frame with unknown packet ID: 127");
}

TEST(VescModelTest, VescModelDecodesStatusPacketsOneToThreeTest) {
    const auto model = VescModel{DUMMY_ID};

    auto result = model.decode(make_frame(
        VescModel::PacketStatus::ID,
        {std::byte{0x00}, std::byte{0x00}, std::byte{0x05}, std::byte{0x78}, std::byte{0x00},
         std::byte{0x7B}, std::byte{0x01}, std::byte{0xF4}}));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    const auto & status = std::get<VescModel::PacketStatus>(result.value().value());
    EXPECT_DOUBLE_EQ(status.erpm, 1400.0);
    EXPECT_DOUBLE_EQ(status.current, 12.3);
    EXPECT_DOUBLE_EQ(status.duty, 0.5);

    result = model.decode(make_frame(VescModel::PacketStatus2::ID, two_int32s(12500, -5000)));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    const auto & status2 = std::get<VescModel::PacketStatus2>(result.value().value());
    EXPECT_DOUBLE_EQ(status2.amp_hour, 1.25);
    EXPECT_DOUBLE_EQ(status2.amp_hour_charge, -0.5);

    result = model.decode(make_frame(VescModel::PacketStatus3::ID, two_int32s(125000, -20000)));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    const auto & status3 = std::get<VescModel::PacketStatus3>(result.value().value());
    EXPECT_DOUBLE_EQ(status3.watt_hour, 12.5);
    EXPECT_DOUBLE_EQ(status3.watt_hour_charge, -2.0);
}

TEST(VescModelTest, VescModelDecodesStatusPacketsFourToSixTest) {
    const auto model = VescModel{DUMMY_ID};

    auto result =
        model.decode(make_frame(VescModel::PacketStatus4::ID, four_int16s(250, -105, 123, 4500)));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    const auto & status4 = std::get<VescModel::PacketStatus4>(result.value().value());
    EXPECT_DOUBLE_EQ(status4.temp_fet, 25.0);
    EXPECT_DOUBLE_EQ(status4.temp_motor, -10.5);
    EXPECT_DOUBLE_EQ(status4.current_in, 12.3);
    EXPECT_DOUBLE_EQ(status4.pid_pos, 90.0);

    result = model.decode(make_frame(VescModel::PacketStatus5::ID, int32_and_int16(600, 485)));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    const auto & status5 = std::get<VescModel::PacketStatus5>(result.value().value());
    EXPECT_DOUBLE_EQ(status5.tachometer, 100.0);
    EXPECT_DOUBLE_EQ(status5.volts_in, 48.5);

    result =
        model.decode(make_frame(VescModel::PacketStatus6::ID, four_int16s(1200, 2300, 3400, 500)));
    ASSERT_TRUE(result);
    ASSERT_TRUE(result.value());
    const auto & status6 = std::get<VescModel::PacketStatus6>(result.value().value());
    EXPECT_DOUBLE_EQ(status6.adc1, 1.2);
    EXPECT_DOUBLE_EQ(status6.adc2, 2.3);
    EXPECT_DOUBLE_EQ(status6.adc3, 3.4);
    EXPECT_DOUBLE_EQ(status6.ppm, 0.5);
}

}  // namespace sinsei_umiusi_control::test::hardware_model::can::vesc

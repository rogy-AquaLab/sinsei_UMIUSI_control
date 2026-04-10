#include <array>
#include <cstring>
#include <rcpputils/tl_expected/expected.hpp>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "mock/i2c.hpp"
#include "sinsei_umiusi_control/hardware_model/imu_model.hpp"

namespace sinsei_umiusi_control::test::hardware_model::imu {

using ::testing::Return;

namespace {

constexpr auto _ = ::testing::_;

}

// TEST(ImuModelTest, OnInitSuccess) {
//     auto mock_i2c = std::make_unique<sinsei_umiusi_control::test::mock::I2c>();

//     EXPECT_CALL(*mock_i2c, open(_))
//         .Times(1)
//         .WillOnce(Return(
//             tl::expected<void, sinsei_umiusi_control::hardware_model::interface::I2c::Error>{}));
//     EXPECT_CALL(*mock_i2c, read_byte_data(_))
//         .WillRepeatedly(Return(
//             tl::expected<std::byte, sinsei_umiusi_control::hardware_model::interface::I2c::Error>{
//                 std::byte{0xA0}}));
//     EXPECT_CALL(*mock_i2c, write_byte_data(_, _))
//         .WillRepeatedly(Return(
//             tl::expected<void, sinsei_umiusi_control::hardware_model::interface::I2c::Error>{}));

//     sinsei_umiusi_control::hardware_model::ImuModel imu_model(std::move(mock_i2c));
//     const auto res = imu_model.on_init();

//     ASSERT_TRUE(res);
// }

// TEST(ImuModelTest, OnInitFailOnI2cOpen) {
//     auto mock_i2c = std::make_unique<sinsei_umiusi_control::test::mock::I2c>();

//     EXPECT_CALL(*mock_i2c, open(_))
//         .Times(1)
//         .WillOnce(Return(tl::make_unexpected(
//             sinsei_umiusi_control::hardware_model::interface::I2c::Error::OpenFailed)));

//     sinsei_umiusi_control::hardware_model::ImuModel imu_model(std::move(mock_i2c));
//     const auto res = imu_model.on_init();

//     ASSERT_FALSE(res);
// }

// TEST(ImuModelTest, OnReadSuccess) {
//     auto mock_i2c = std::make_unique<sinsei_umiusi_control::test::mock::I2c>();

//     static constexpr auto START_ADDR =
//         sinsei_umiusi_control::hardware_model::ImuModel::GYRO_DATA_X_LSB_ADDR;
//     static constexpr auto END_ADDR =
//         sinsei_umiusi_control::hardware_model::ImuModel::TEMP_ADDR;
//     static constexpr size_t FRAME_LENGTH = END_ADDR - START_ADDR + 1;

//     auto mock_data = std::array<std::byte, FRAME_LENGTH>{};
//     const auto write_s16 = [&mock_data](size_t offset, int16_t value) {
//         const auto raw = static_cast<uint16_t>(value);
//         mock_data[offset] = std::byte{static_cast<uint8_t>(raw & 0xFF)};
//         mock_data[offset + 1] = std::byte{static_cast<uint8_t>((raw >> 8) & 0xFF)};
//     };
//     const auto offset_quat = static_cast<size_t>(
//         sinsei_umiusi_control::hardware_model::ImuModel::QUATERNION_DATA_W_LSB_ADDR -
//         START_ADDR);
//     const auto offset_linear_accel = static_cast<size_t>(
//         sinsei_umiusi_control::hardware_model::ImuModel::LINEAR_ACCEL_DATA_X_LSB_ADDR -
//         START_ADDR);
//     const auto offset_temp = static_cast<size_t>(END_ADDR - START_ADDR);

//     write_s16(0, 0x0100);
//     write_s16(2, 0x0200);
//     write_s16(4, 0x0300);
//     write_s16(offset_quat + 0, 0x1000);
//     write_s16(offset_quat + 2, 0x0800);
//     write_s16(offset_quat + 4, 0x0400);
//     write_s16(offset_quat + 6, 0x0200);
//     write_s16(offset_linear_accel + 0, 0x0064);
//     write_s16(offset_linear_accel + 2, 0x00C8);
//     write_s16(offset_linear_accel + 4, 0x012C);
//     mock_data[offset_temp] = std::byte{0x05};

//     EXPECT_CALL(*mock_i2c, transfer(_))
//         .Times(1)
//         .WillOnce(testing::Invoke(
//             [&mock_data](
//                 const std::vector<sinsei_umiusi_control::hardware_model::interface::I2cMessage> &
//                     msgs) {
//                 EXPECT_EQ(msgs.size(), 2U);
//                 EXPECT_EQ(
//                     msgs[0].direction,
//                     sinsei_umiusi_control::hardware_model::interface::I2cDirection::Write);
//                 EXPECT_EQ(
//                     msgs[1].direction,
//                     sinsei_umiusi_control::hardware_model::interface::I2cDirection::Read);
//                 EXPECT_EQ(msgs[1].length, mock_data.size());
//                 std::memcpy(msgs[1].data, mock_data.data(), mock_data.size());
//                 return tl::expected<
//                     void, sinsei_umiusi_control::hardware_model::interface::I2c::Error>{};
//             }));

//     sinsei_umiusi_control::hardware_model::ImuModel imu_model(std::move(mock_i2c));
//     const auto res = imu_model.on_read();

//     ASSERT_TRUE(res);
//     // NOTE: 値の中身については`imu_bno055.cpp`のテストケースで確認
// }

// TEST(ImuModelTest, OnDestroySuccess) {
//     auto mock_i2c = std::make_unique<sinsei_umiusi_control::test::mock::I2c>();

//     EXPECT_CALL(*mock_i2c, close())
//         .Times(1)
//         .WillOnce(Return(
//             tl::expected<void, sinsei_umiusi_control::hardware_model::interface::I2c::Error>{}));

//     sinsei_umiusi_control::hardware_model::ImuModel imu_model(std::move(mock_i2c));
//     const auto res = imu_model.on_destroy();

//     ASSERT_TRUE(res);
// }

// TEST(ImuModelTest, OnDestroyFail) {
//     auto mock_i2c = std::make_unique<sinsei_umiusi_control::test::mock::I2c>();

//     EXPECT_CALL(*mock_i2c, close())
//         .Times(1)
//         .WillOnce(Return(tl::make_unexpected(
//             sinsei_umiusi_control::hardware_model::interface::I2c::Error::NotOpen)));

//     sinsei_umiusi_control::hardware_model::ImuModel imu_model(std::move(mock_i2c));
//     const auto res = imu_model.on_destroy();

//     ASSERT_FALSE(res);
// }

}  // namespace sinsei_umiusi_control::test::hardware_model::imu

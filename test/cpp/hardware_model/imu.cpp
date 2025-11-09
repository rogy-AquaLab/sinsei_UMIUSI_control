#include <rcpputils/tl_expected/expected.hpp>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "mock/gpio.hpp"
#include "sinsei_umiusi_control/hardware_model/imu_model.hpp"

namespace sinsei_umiusi_control::test::hardware_model::imu {

using ::testing::DoAll;
using ::testing::Return;

namespace {

constexpr auto _ = ::testing::_;

}

TEST(ImuModelTest, OnInitSuccess) {
    auto mock_gpio = std::make_unique<sinsei_umiusi_control::test::mock::Gpio>();

    EXPECT_CALL(*mock_gpio, i2c_open(_))
        .Times(1)
        .WillOnce(Return(
            tl::expected<void, sinsei_umiusi_control::hardware_model::interface::Gpio::Error>{}));
    EXPECT_CALL(*mock_gpio, i2c_read_byte_data(_))
        .WillRepeatedly(Return(
            tl::expected<std::byte, sinsei_umiusi_control::hardware_model::interface::Gpio::Error>{
                std::byte{0xA0}}));
    EXPECT_CALL(*mock_gpio, i2c_write_byte_data(_, _))
        .WillRepeatedly(Return(
            tl::expected<void, sinsei_umiusi_control::hardware_model::interface::Gpio::Error>{}));

    sinsei_umiusi_control::hardware_model::ImuModel imu_model(std::move(mock_gpio));
    const auto res = imu_model.on_init();

    ASSERT_TRUE(res);
}

TEST(ImuModelTest, OnInitFailOnI2cOpen) {
    auto mock_gpio = std::make_unique<sinsei_umiusi_control::test::mock::Gpio>();

    EXPECT_CALL(*mock_gpio, i2c_open(_))
        .Times(1)
        .WillOnce(Return(tl::make_unexpected(
            sinsei_umiusi_control::hardware_model::interface::Gpio::Error::I2cOpenFailed)));

    sinsei_umiusi_control::hardware_model::ImuModel imu_model(std::move(mock_gpio));
    const auto res = imu_model.on_init();

    ASSERT_FALSE(res);
}

TEST(ImuModelTest, OnReadSuccess) {
    auto mock_gpio = std::make_unique<sinsei_umiusi_control::test::mock::Gpio>();

    constexpr std::byte MOCK_DATA_QUAT[8] = {std::byte{0x00}, std::byte{0x10},   // w
                                             std::byte{0x00}, std::byte{0x20},   // x
                                             std::byte{0x00}, std::byte{0x30},   // y
                                             std::byte{0x00}, std::byte{0x40}};  // z
    constexpr std::byte MOCK_DATA_VEC[6] = {std::byte{0x00}, std::byte{0x64},    // x
                                            std::byte{0x00}, std::byte{0xC8},    // y
                                            std::byte{0x01}, std::byte{0x2C}};   // z

    EXPECT_CALL(*mock_gpio, i2c_read_block_data(_, _, 8))
        .Times(1)
        .WillOnce(DoAll(
            testing::SetArrayArgument<1>(MOCK_DATA_QUAT, MOCK_DATA_QUAT + 8),
            Return(tl::expected<
                   void, sinsei_umiusi_control::hardware_model::interface::Gpio::Error>{})));
    EXPECT_CALL(*mock_gpio, i2c_read_block_data(_, _, 6))
        .Times(2)
        .WillRepeatedly(DoAll(
            testing::SetArrayArgument<1>(MOCK_DATA_VEC, MOCK_DATA_VEC + 6),
            Return(tl::expected<
                   void, sinsei_umiusi_control::hardware_model::interface::Gpio::Error>{})));
    EXPECT_CALL(*mock_gpio, i2c_read_byte_data(_))
        .Times(1)
        .WillOnce(Return(
            tl::expected<std::byte, sinsei_umiusi_control::hardware_model::interface::Gpio::Error>{
                std::byte{0x03}}));

    sinsei_umiusi_control::hardware_model::ImuModel imu_model(std::move(mock_gpio));
    const auto res = imu_model.on_read();

    ASSERT_TRUE(res);
    // NOTE: 値の中身については`imu/bno055_model.cpp`のテストケースで確認
}

TEST(ImuModelTest, OnDestroySuccess) {
    auto mock_gpio = std::make_unique<sinsei_umiusi_control::test::mock::Gpio>();

    EXPECT_CALL(*mock_gpio, i2c_close())
        .Times(1)
        .WillOnce(Return(
            tl::expected<void, sinsei_umiusi_control::hardware_model::interface::Gpio::Error>{}));

    sinsei_umiusi_control::hardware_model::ImuModel imu_model(std::move(mock_gpio));
    const auto res = imu_model.on_destroy();

    ASSERT_TRUE(res);
}

TEST(ImuModelTest, OnDestroyFail) {
    auto mock_gpio = std::make_unique<sinsei_umiusi_control::test::mock::Gpio>();

    EXPECT_CALL(*mock_gpio, i2c_close())
        .Times(1)
        .WillOnce(Return(tl::make_unexpected(
            sinsei_umiusi_control::hardware_model::interface::Gpio::Error::I2cNotOpen)));

    sinsei_umiusi_control::hardware_model::ImuModel imu_model(std::move(mock_gpio));
    const auto res = imu_model.on_destroy();

    ASSERT_FALSE(res);
}

}  // namespace sinsei_umiusi_control::test::hardware_model::imu

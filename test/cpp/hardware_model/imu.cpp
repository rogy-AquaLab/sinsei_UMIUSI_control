#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>

#include "mock/i2c.hpp"
#include "sinsei_umiusi_control/hardware_model/imu/bno055_model.hpp"
#include "sinsei_umiusi_control/hardware_model/imu_model.hpp"

namespace sinsei_umiusi_control::test::hardware_model::imu {

using ::testing::Invoke;
using ::testing::Return;

namespace {

using sinsei_umiusi_control::hardware_model::ImuModel;
using sinsei_umiusi_control::hardware_model::imu::Bno055Model;
using sinsei_umiusi_control::hardware_model::interface::I2cDirection;
using sinsei_umiusi_control::hardware_model::interface::I2cMessage;

auto expect_write_reg(
    const I2cMessage * msgs, std::size_t size,
    sinsei_umiusi_control::hardware_model::interface::I2cRegisterAddr reg,
    std::byte value) -> void {
    ASSERT_NE(msgs, nullptr);
    ASSERT_EQ(size, 1U);
    EXPECT_EQ(msgs[0].address.value, Bno055Model::ADDRESS.value);
    EXPECT_EQ(msgs[0].direction, I2cDirection::Write);
    ASSERT_EQ(msgs[0].buffer.length, 2U);
    EXPECT_EQ(msgs[0].buffer.data[0], reg.value);
    EXPECT_EQ(msgs[0].buffer.data[1], value);
}

auto expect_read_reg(
    const I2cMessage * msgs, std::size_t size,
    sinsei_umiusi_control::hardware_model::interface::I2cRegisterAddr reg,
    std::size_t read_size) -> void {
    ASSERT_NE(msgs, nullptr);
    ASSERT_EQ(size, 2U);
    EXPECT_EQ(msgs[0].address.value, Bno055Model::ADDRESS.value);
    EXPECT_EQ(msgs[0].direction, I2cDirection::Write);
    ASSERT_EQ(msgs[0].buffer.length, 1U);
    EXPECT_EQ(msgs[0].buffer.data[0], reg.value);
    EXPECT_EQ(msgs[1].address.value, Bno055Model::ADDRESS.value);
    EXPECT_EQ(msgs[1].direction, I2cDirection::Read);
    EXPECT_EQ(msgs[1].buffer.length, read_size);
}

auto return_chip_id(const I2cMessage * msgs, std::size_t size, std::byte value)
    -> tl::expected<void, std::string> {
    expect_read_reg(msgs, size, Bno055Model::CHIP_ID_ADDR, 1U);
    msgs[1].buffer.data[0] = value;
    return {};
}

auto expect_read_frame(const I2cMessage * msgs, std::size_t size) -> void {
    expect_read_reg(msgs, size, Bno055Model::FRAME_START, Bno055Model::FRAME_LENGTH);
}

}  // namespace

TEST(ImuModelTest, OnInitSuccess) {
    auto mock_i2c = std::make_unique<mock::I2c>();

    EXPECT_CALL(*mock_i2c, open()).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*mock_i2c, transfer(testing::_, testing::_))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            return return_chip_id(msgs, size, std::byte{Bno055Model::CHIP_ID});
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            expect_write_reg(
                msgs, size, Bno055Model::OPR_MODE_ADDR, Bno055Model::OPERATION_MODE_CONFIG);
            return tl::expected<void, std::string>{};
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            expect_write_reg(msgs, size, Bno055Model::SYS_TRIGGER_ADDR, std::byte{0x20});
            return tl::expected<void, std::string>{};
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            return return_chip_id(msgs, size, std::byte{Bno055Model::CHIP_ID});
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            expect_write_reg(
                msgs, size, Bno055Model::PWR_MODE_ADDR, Bno055Model::POWER_MODE_NORMAL);
            return tl::expected<void, std::string>{};
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            expect_write_reg(msgs, size, Bno055Model::PAGE_ID_ADDR, std::byte{0x00});
            return tl::expected<void, std::string>{};
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            expect_write_reg(msgs, size, Bno055Model::SYS_TRIGGER_ADDR, std::byte{0x00});
            return tl::expected<void, std::string>{};
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            expect_write_reg(
                msgs, size, Bno055Model::OPR_MODE_ADDR, Bno055Model::OPERATION_MODE_NDOF);
            return tl::expected<void, std::string>{};
        }));

    ImuModel imu_model(std::move(mock_i2c));
    const auto res = imu_model.on_init();

    ASSERT_TRUE(res);
}

TEST(ImuModelTest, OnInitFailOnI2cOpen) {
    auto mock_i2c = std::make_unique<mock::I2c>();

    EXPECT_CALL(*mock_i2c, open()).WillOnce(Return(tl::make_unexpected("open failed")));

    ImuModel imu_model(std::move(mock_i2c));
    const auto res = imu_model.on_init();

    ASSERT_FALSE(res);
}

TEST(ImuModelTest, OnReadSuccess) {
    auto mock_i2c = std::make_unique<mock::I2c>();

    auto read_buffer = std::array<std::byte, Bno055Model::FRAME_LENGTH>{};
    constexpr std::byte MOCK_DATA_QUAT[8] = {std::byte{0x00}, std::byte{0x10}, std::byte{0x00},
                                             std::byte{0x20}, std::byte{0x00}, std::byte{0x30},
                                             std::byte{0x00}, std::byte{0x40}};
    constexpr std::byte MOCK_DATA_GYRO[6] = {std::byte{0x00}, std::byte{0x64}, std::byte{0x00},
                                             std::byte{0xC8}, std::byte{0x01}, std::byte{0x2C}};
    constexpr std::byte MOCK_DATA_ACCEL[6] = {std::byte{0x10}, std::byte{0x00}, std::byte{0x20},
                                              std::byte{0x00}, std::byte{0x30}, std::byte{0x00}};
    constexpr auto MOCK_TEMP = std::byte{0x83};

    std::memcpy(
        read_buffer.data() + Bno055Model::OFFSET_GYRO, MOCK_DATA_GYRO, sizeof(MOCK_DATA_GYRO));
    std::memcpy(
        read_buffer.data() + Bno055Model::OFFSET_QUAT, MOCK_DATA_QUAT, sizeof(MOCK_DATA_QUAT));
    std::memcpy(
        read_buffer.data() + Bno055Model::OFFSET_LINEAR_ACCEL, MOCK_DATA_ACCEL,
        sizeof(MOCK_DATA_ACCEL));
    read_buffer[Bno055Model::OFFSET_TEMP] = MOCK_TEMP;

    EXPECT_CALL(*mock_i2c, transfer(testing::_, testing::_))
        .WillOnce(Invoke([&](const I2cMessage * msgs, std::size_t size) {
            expect_read_frame(msgs, size);
            std::memcpy(msgs[1].buffer.data, read_buffer.data(), read_buffer.size());
            return tl::expected<void, std::string>{};
        }));

    ImuModel imu_model(std::move(mock_i2c));
    const auto res = imu_model.on_read();

    ASSERT_TRUE(res);
}

TEST(ImuModelTest, OnDestroySuccess) {
    auto mock_i2c = std::make_unique<mock::I2c>();

    EXPECT_CALL(*mock_i2c, close()).WillOnce(Return(tl::expected<void, std::string>{}));

    ImuModel imu_model(std::move(mock_i2c));
    const auto res = imu_model.on_destroy();

    ASSERT_TRUE(res);
}

TEST(ImuModelTest, OnDestroyFail) {
    auto mock_i2c = std::make_unique<mock::I2c>();

    EXPECT_CALL(*mock_i2c, close()).WillOnce(Return(tl::make_unexpected("not open")));

    ImuModel imu_model(std::move(mock_i2c));
    const auto res = imu_model.on_destroy();

    ASSERT_FALSE(res);
}

}  // namespace sinsei_umiusi_control::test::hardware_model::imu

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <array>
#include <boost/math/constants/constants.hpp>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>

#include "mock/i2c.hpp"
#include "sinsei_umiusi_control/hardware_model/imu_model.hpp"

namespace suchm = sinsei_umiusi_control::hardware_model;

using testing::HasSubstr;
using testing::InSequence;
using testing::Invoke;
using testing::Return;

namespace sinsei_umiusi_control::test::hardware_model::imu {

namespace {

constexpr auto _ = testing::_;

using ImuModel = suchm::ImuModel;
using I2cDirection = suchm::interface::I2cDirection;
using I2cMessage = suchm::interface::I2cMessage;
using I2cRegisterAddr = suchm::interface::I2cRegisterAddr;

auto expect_write_reg(const I2cMessage * msgs, std::size_t size, I2cRegisterAddr reg, std::byte value)
    -> void {
    ASSERT_NE(msgs, nullptr);
    ASSERT_EQ(size, 1U);
    EXPECT_EQ(msgs[0].address.value, ImuModel::ADDRESS.value);
    EXPECT_EQ(msgs[0].direction, I2cDirection::Write);
    ASSERT_EQ(msgs[0].buffer.length, 2U);
    EXPECT_EQ(msgs[0].buffer.data[0], reg.value);
    EXPECT_EQ(msgs[0].buffer.data[1], value);
}

auto expect_read_reg(const I2cMessage * msgs, std::size_t size, I2cRegisterAddr reg, std::size_t read_size)
    -> void {
    ASSERT_NE(msgs, nullptr);
    ASSERT_EQ(size, 2U);
    EXPECT_EQ(msgs[0].address.value, ImuModel::ADDRESS.value);
    EXPECT_EQ(msgs[0].direction, I2cDirection::Write);
    ASSERT_EQ(msgs[0].buffer.length, 1U);
    EXPECT_EQ(msgs[0].buffer.data[0], reg.value);
    EXPECT_EQ(msgs[1].address.value, ImuModel::ADDRESS.value);
    EXPECT_EQ(msgs[1].direction, I2cDirection::Read);
    EXPECT_EQ(msgs[1].buffer.length, read_size);
}

auto read_chip_id_action(std::byte value) {
    return [value](const I2cMessage * msgs, std::size_t size) -> tl::expected<void, std::string> {
        expect_read_reg(msgs, size, ImuModel::CHIP_ID_ADDR, 1U);
        msgs[1].buffer.data[0] = value;
        return {};
    };
}

auto fail_read_chip_id_action(std::string error) {
    return [error = std::move(error)](
               const I2cMessage * msgs, std::size_t size) -> tl::expected<void, std::string> {
        expect_read_reg(msgs, size, ImuModel::CHIP_ID_ADDR, 1U);
        return tl::make_unexpected(error);
    };
}

auto write_reg_action(I2cRegisterAddr reg, std::byte value) {
    return [reg, value](const I2cMessage * msgs, std::size_t size) -> tl::expected<void, std::string> {
        expect_write_reg(msgs, size, reg, value);
        return {};
    };
}

auto fail_write_reg_action(I2cRegisterAddr reg, std::byte value, std::string error) {
    return [reg, value, error = std::move(error)](
               const I2cMessage * msgs, std::size_t size) -> tl::expected<void, std::string> {
        expect_write_reg(msgs, size, reg, value);
        return tl::make_unexpected(error);
    };
}

auto read_frame_action(const std::array<std::byte, ImuModel::FRAME_LENGTH> & frame) {
    return [&frame](const I2cMessage * msgs, std::size_t size) -> tl::expected<void, std::string> {
        expect_read_reg(msgs, size, ImuModel::FRAME_START, ImuModel::FRAME_LENGTH);
        std::memcpy(msgs[1].buffer.data, frame.data(), frame.size());
        return {};
    };
}

auto fail_read_frame_action(std::string error) {
    return [error = std::move(error)](
               const I2cMessage * msgs, std::size_t size) -> tl::expected<void, std::string> {
        expect_read_reg(msgs, size, ImuModel::FRAME_START, ImuModel::FRAME_LENGTH);
        return tl::make_unexpected(error);
    };
}

auto set_s16(std::byte * bytes, int16_t value) -> void {
    const auto raw = static_cast<uint16_t>(value);
    bytes[0] = static_cast<std::byte>(raw & 0xFF);
    bytes[1] = static_cast<std::byte>((raw >> 8) & 0xFF);
}

}  // namespace

TEST(ImuModelTest, OnInitSuccess) {
    auto sequence = InSequence{};
    auto i2c = std::make_unique<mock::I2c>();
    auto * i2c_raw = i2c.get();

    EXPECT_CALL(*i2c_raw, open()).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*i2c_raw, transfer(_, _))
        .WillOnce(Invoke(read_chip_id_action(std::byte{0x00})))
        .WillOnce(Invoke(read_chip_id_action(std::byte{ImuModel::CHIP_ID})))
        .WillOnce(
            Invoke(write_reg_action(ImuModel::OPR_MODE_ADDR, ImuModel::OPERATION_MODE_CONFIG)))
        .WillOnce(Invoke(write_reg_action(ImuModel::SYS_TRIGGER_ADDR, std::byte{0x20})))
        .WillOnce(Invoke(read_chip_id_action(std::byte{0x00})))
        .WillOnce(Invoke(read_chip_id_action(std::byte{ImuModel::CHIP_ID})))
        .WillOnce(Invoke(write_reg_action(ImuModel::PWR_MODE_ADDR, ImuModel::POWER_MODE_NORMAL)))
        .WillOnce(Invoke(write_reg_action(ImuModel::PAGE_ID_ADDR, std::byte{0x00})))
        .WillOnce(Invoke(write_reg_action(ImuModel::SYS_TRIGGER_ADDR, std::byte{0x00})))
        .WillOnce(
            Invoke(write_reg_action(ImuModel::OPR_MODE_ADDR, ImuModel::OPERATION_MODE_NDOF)));

    auto imu_model = ImuModel{std::move(i2c)};
    const auto result = imu_model.on_init();

    ASSERT_TRUE(result) << result.error();
}

TEST(ImuModelTest, OnInitFailsWhenOpenFails) {
    auto i2c = std::make_unique<mock::I2c>();
    auto * i2c_raw = i2c.get();

    EXPECT_CALL(*i2c_raw, open()).WillOnce(Return(tl::make_unexpected("open failed")));

    auto imu_model = ImuModel{std::move(i2c)};
    const auto result = imu_model.on_init();

    ASSERT_FALSE(result);
    EXPECT_THAT(result.error(), HasSubstr("Failed to open I2C bus for BNO055"));
}

TEST(ImuModelTest, OnInitFailsWhenDeviceVerificationNeverSucceeds) {
    auto sequence = InSequence{};
    auto i2c = std::make_unique<mock::I2c>();
    auto * i2c_raw = i2c.get();

    EXPECT_CALL(*i2c_raw, open()).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*i2c_raw, transfer(_, _)).Times(2).WillRepeatedly(Invoke(fail_read_chip_id_action("read failed")));

    auto imu_model = ImuModel{std::move(i2c)};
    const auto result = imu_model.on_init();

    ASSERT_FALSE(result);
    EXPECT_THAT(result.error(), HasSubstr("Failed to verify BNO055 device"));
    EXPECT_THAT(result.error(), HasSubstr("device was not ready after 2 attempts"));
}

TEST(ImuModelTest, OnInitFailsWhenResetWriteFails) {
    auto sequence = InSequence{};
    auto i2c = std::make_unique<mock::I2c>();
    auto * i2c_raw = i2c.get();

    EXPECT_CALL(*i2c_raw, open()).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*i2c_raw, transfer(_, _))
        .WillOnce(Invoke(read_chip_id_action(std::byte{ImuModel::CHIP_ID})))
        .WillOnce(Invoke(fail_write_reg_action(
            ImuModel::OPR_MODE_ADDR, ImuModel::OPERATION_MODE_CONFIG, "config failed")));

    auto imu_model = ImuModel{std::move(i2c)};
    const auto result = imu_model.on_init();

    ASSERT_FALSE(result);
    EXPECT_THAT(result.error(), HasSubstr("Failed to reset BNO055 device"));
    EXPECT_THAT(result.error(), HasSubstr("Failed to set BNO055 to CONFIG mode"));
}

TEST(ImuModelTest, OnInitFailsWhenDeviceDoesNotComeBackAfterReset) {
    auto sequence = InSequence{};
    auto i2c = std::make_unique<mock::I2c>();
    auto * i2c_raw = i2c.get();

    EXPECT_CALL(*i2c_raw, open()).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*i2c_raw, transfer(_, _))
        .WillOnce(Invoke(read_chip_id_action(std::byte{ImuModel::CHIP_ID})))
        .WillOnce(
            Invoke(write_reg_action(ImuModel::OPR_MODE_ADDR, ImuModel::OPERATION_MODE_CONFIG)))
        .WillOnce(Invoke(write_reg_action(ImuModel::SYS_TRIGGER_ADDR, std::byte{0x20})));
    EXPECT_CALL(*i2c_raw, transfer(_, _))
        .Times(100)
        .WillRepeatedly(Invoke(read_chip_id_action(std::byte{0x00})));

    auto imu_model = ImuModel{std::move(i2c)};
    const auto result = imu_model.on_init();

    ASSERT_FALSE(result);
    EXPECT_THAT(result.error(), HasSubstr("BNO055 did not restart within timeout period"));
    EXPECT_THAT(result.error(), HasSubstr("device was not ready after 100 attempts"));
}

TEST(ImuModelTest, OnInitFailsWhenConfigureFails) {
    auto sequence = InSequence{};
    auto i2c = std::make_unique<mock::I2c>();
    auto * i2c_raw = i2c.get();

    EXPECT_CALL(*i2c_raw, open()).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*i2c_raw, transfer(_, _))
        .WillOnce(Invoke(read_chip_id_action(std::byte{ImuModel::CHIP_ID})))
        .WillOnce(
            Invoke(write_reg_action(ImuModel::OPR_MODE_ADDR, ImuModel::OPERATION_MODE_CONFIG)))
        .WillOnce(Invoke(write_reg_action(ImuModel::SYS_TRIGGER_ADDR, std::byte{0x20})))
        .WillOnce(Invoke(read_chip_id_action(std::byte{ImuModel::CHIP_ID})))
        .WillOnce(Invoke(fail_write_reg_action(
            ImuModel::PWR_MODE_ADDR, ImuModel::POWER_MODE_NORMAL, "power mode failed")));

    auto imu_model = ImuModel{std::move(i2c)};
    const auto result = imu_model.on_init();

    ASSERT_FALSE(result);
    EXPECT_THAT(result.error(), HasSubstr("Failed to configure BNO055 device"));
    EXPECT_THAT(result.error(), HasSubstr("Failed to set BNO055 to NORMAL power mode"));
}

TEST(ImuModelTest, OnDestroySuccess) {
    auto i2c = std::make_unique<mock::I2c>();
    auto * i2c_raw = i2c.get();

    EXPECT_CALL(*i2c_raw, close()).WillOnce(Return(tl::expected<void, std::string>{}));

    auto imu_model = ImuModel{std::move(i2c)};
    const auto result = imu_model.on_destroy();

    ASSERT_TRUE(result) << result.error();
}

TEST(ImuModelTest, OnDestroyFailsWhenI2cCloseFails) {
    auto i2c = std::make_unique<mock::I2c>();
    auto * i2c_raw = i2c.get();

    EXPECT_CALL(*i2c_raw, close()).WillOnce(Return(tl::make_unexpected("not open")));

    auto imu_model = ImuModel{std::move(i2c)};
    const auto result = imu_model.on_destroy();

    ASSERT_FALSE(result);
    EXPECT_THAT(result.error(), HasSubstr("Failed to close I2C bus"));
}

TEST(ImuModelTest, OnReadSuccess) {
    constexpr auto PI = boost::math::constants::pi<double>();

    auto frame = std::array<std::byte, ImuModel::FRAME_LENGTH>{};
    auto i2c = std::make_unique<mock::I2c>();
    auto * i2c_raw = i2c.get();

    set_s16(frame.data() + ImuModel::OFFSET_GYRO + 0, 16);
    set_s16(frame.data() + ImuModel::OFFSET_GYRO + 2, -32);
    set_s16(frame.data() + ImuModel::OFFSET_GYRO + 4, 48);

    set_s16(frame.data() + ImuModel::OFFSET_QUAT + 0, 16384);
    set_s16(frame.data() + ImuModel::OFFSET_QUAT + 2, 8192);
    set_s16(frame.data() + ImuModel::OFFSET_QUAT + 4, -4096);
    set_s16(frame.data() + ImuModel::OFFSET_QUAT + 6, 2048);

    set_s16(frame.data() + ImuModel::OFFSET_LINEAR_ACCEL + 0, 100);
    set_s16(frame.data() + ImuModel::OFFSET_LINEAR_ACCEL + 2, -250);
    set_s16(frame.data() + ImuModel::OFFSET_LINEAR_ACCEL + 4, 30);

    frame[ImuModel::OFFSET_TEMP] = std::byte{0xFB};

    EXPECT_CALL(*i2c_raw, transfer(_, _)).WillOnce(Invoke(read_frame_action(frame)));

    auto imu_model = ImuModel{std::move(i2c)};
    const auto result = imu_model.on_read();

    ASSERT_TRUE(result) << result.error();

    const auto & [quaternion, acceleration, angular_velocity, temperature] = result.value();

    EXPECT_DOUBLE_EQ(quaternion.x, 0.5);
    EXPECT_DOUBLE_EQ(quaternion.y, -0.25);
    EXPECT_DOUBLE_EQ(quaternion.z, 0.125);
    EXPECT_DOUBLE_EQ(quaternion.w, 1.0);

    EXPECT_DOUBLE_EQ(acceleration.x, 1.0);
    EXPECT_DOUBLE_EQ(acceleration.y, -2.5);
    EXPECT_DOUBLE_EQ(acceleration.z, 0.3);

    EXPECT_NEAR(angular_velocity.x, PI / 180.0, 1e-12);
    EXPECT_NEAR(angular_velocity.y, -2.0 * PI / 180.0, 1e-12);
    EXPECT_NEAR(angular_velocity.z, 3.0 * PI / 180.0, 1e-12);

    EXPECT_EQ(temperature.value, -5);
}

TEST(ImuModelTest, OnReadFailsWhenI2cTransferFails) {
    auto i2c = std::make_unique<mock::I2c>();
    auto * i2c_raw = i2c.get();

    EXPECT_CALL(*i2c_raw, transfer(_, _)).WillOnce(Invoke(fail_read_frame_action("read failed")));

    auto imu_model = ImuModel{std::move(i2c)};
    const auto result = imu_model.on_read();

    ASSERT_FALSE(result);
    EXPECT_THAT(result.error(), HasSubstr("I2C read error"));
}

}  // namespace sinsei_umiusi_control::test::hardware_model::imu

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>

#include "mock/gpio.hpp"
#include "sinsei_umiusi_control/hardware_model/imu/bno055_model.hpp"
#include "sinsei_umiusi_control/hardware_model/interface/gpio.hpp"
#include "sinsei_umiusi_control/state/imu.hpp"

using namespace sinsei_umiusi_control::hardware_model;

using ::testing::AtLeast;
using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Return;

namespace {

constexpr auto _ = ::testing::_;

}

namespace sinsei_umiusi_control::test::hardware_model::bno055 {

static constexpr auto ADDRESS = ::sinsei_umiusi_control::hardware_model::imu::Bno055Model::ADDRESS;
static constexpr auto ID = ::sinsei_umiusi_control::hardware_model::imu::Bno055Model::ID;
static constexpr auto CHIP_ID_ADDR =
    ::sinsei_umiusi_control::hardware_model::imu::Bno055Model::CHIP_ID_ADDR;
static constexpr auto OPR_MODE_ADDR =
    ::sinsei_umiusi_control::hardware_model::imu::Bno055Model::OPR_MODE_ADDR;
static constexpr auto SYS_TRIGGER_ADDR =
    ::sinsei_umiusi_control::hardware_model::imu::Bno055Model::SYS_TRIGGER_ADDR;
static constexpr auto PWR_MODE_ADDR =
    ::sinsei_umiusi_control::hardware_model::imu::Bno055Model::PWR_MODE_ADDR;
static constexpr auto PAGE_ID_ADDR =
    ::sinsei_umiusi_control::hardware_model::imu::Bno055Model::PAGE_ID_ADDR;
static constexpr auto OPERATION_MODE_CONFIG =
    ::sinsei_umiusi_control::hardware_model::imu::Bno055Model::OPERATION_MODE_CONFIG;
static constexpr auto OPERATION_MODE_NDOF =
    ::sinsei_umiusi_control::hardware_model::imu::Bno055Model::OPERATION_MODE_NDOF;
static constexpr auto POWER_MODE_NORMAL =
    ::sinsei_umiusi_control::hardware_model::imu::Bno055Model::POWER_MODE_NORMAL;
static constexpr auto ACCEL_ADDR =
    ::sinsei_umiusi_control::hardware_model::imu::Bno055Model::LINEAR_ACCEL_DATA_X_LSB_ADDR;
static constexpr auto ANGVEL_ADDR =
    ::sinsei_umiusi_control::hardware_model::imu::Bno055Model::GYRO_DATA_X_LSB_ADDR;
static constexpr auto TEMP_ADDR =
    ::sinsei_umiusi_control::hardware_model::imu::Bno055Model::TEMP_ADDR;
static constexpr auto QUAT_ADDR =
    ::sinsei_umiusi_control::hardware_model::imu::Bno055Model::QUATERNION_DATA_W_LSB_ADDR;
static constexpr auto ACCEL_SCALE = 1.0 / 100.0;
static constexpr auto ANGVEL_SCALE = 1.0 / 16.0;
static constexpr auto QUAT_SCALE = 1.0 / (1 << 14);

TEST(Bno055ModelBeginTest, success) {
    auto _ = InSequence{};  // Ensure calls are made in the expected order

    auto gpio = std::make_unique<mock::Gpio>();

    // Activate I2C bus
    EXPECT_CALL(*gpio, i2c_open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Read chip ID
    EXPECT_CALL(*gpio, i2c_read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<std::byte, interface::GpioError>(std::byte{ID})));
    // Set operation mode to CONFIG
    EXPECT_CALL(*gpio, i2c_write_byte_data(OPR_MODE_ADDR, std::byte{OPERATION_MODE_CONFIG}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Trigger reset
    EXPECT_CALL(*gpio, i2c_write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x20}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Wait for reboot (while the chip ID is read again)
    EXPECT_CALL(*gpio, i2c_read_byte_data(CHIP_ID_ADDR))
        .WillOnce(Return(tl::expected<std::byte, interface::GpioError>{std::byte{ID}}));
    // Set power mode to NORMAL
    EXPECT_CALL(*gpio, i2c_write_byte_data(PWR_MODE_ADDR, std::byte{POWER_MODE_NORMAL}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Set page ID to 0
    EXPECT_CALL(*gpio, i2c_write_byte_data(PAGE_ID_ADDR, std::byte{0x0}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Clear SYS_TRIGGER
    EXPECT_CALL(*gpio, i2c_write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x0}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Set operation mode to NDOF
    EXPECT_CALL(*gpio, i2c_write_byte_data(OPR_MODE_ADDR, std::byte{OPERATION_MODE_NDOF}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));

    auto bno055_model = imu::Bno055Model(std::move(gpio));
    auto result = bno055_model.begin();

    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
}

TEST(Bno055ModelBeginTest, fail_on_open) {
    auto gpio = std::make_unique<mock::Gpio>();

    // Activate I2C bus -> FAIL!
    EXPECT_CALL(*gpio, i2c_open(ADDRESS))
        .Times(AtLeast(1))
        .WillRepeatedly(Return(tl::make_unexpected(interface::GpioError::I2cOpenFailed)));

    auto bno055_model = imu::Bno055Model(std::move(gpio));
    auto result = bno055_model.begin();

    ASSERT_FALSE(result);
}

TEST(Bno055ModelBeginTest, fail_on_read_chip_id) {
    auto _ = InSequence{};  // Ensure calls are made in the expected order

    auto gpio = std::make_unique<mock::Gpio>();

    // Activate I2C bus
    EXPECT_CALL(*gpio, i2c_open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    EXPECT_CALL(*gpio, i2c_read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::make_unexpected(interface::GpioError::I2cReadFailed)));

    auto bno055_model = imu::Bno055Model(std::move(gpio));
    auto result = bno055_model.begin();

    ASSERT_FALSE(result);
}

TEST(Bno055ModelBeginTest, fail_on_wrong_chip_id) {
    auto _ = InSequence{};  // Ensure calls are made in the expected order

    auto gpio = std::make_unique<mock::Gpio>();

    // Activate I2C bus
    EXPECT_CALL(*gpio, i2c_open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Read chip ID -> FAIL!
    EXPECT_CALL(*gpio, i2c_read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<std::byte, interface::GpioError>(std::byte{ID + 1})));

    auto bno055_model = imu::Bno055Model(std::move(gpio));
    auto result = bno055_model.begin();

    ASSERT_FALSE(result);
}

TEST(Bno055ModelBeginTest, fail_on_set_opr_mode_config) {
    auto _ = InSequence{};  // Ensure calls are made in the expected order

    auto gpio = std::make_unique<mock::Gpio>();

    // Activate I2C bus
    EXPECT_CALL(*gpio, i2c_open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Read chip ID
    EXPECT_CALL(*gpio, i2c_read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<std::byte, interface::GpioError>(std::byte{ID})));
    // Set operation mode to CONFIG -> FAIL!
    EXPECT_CALL(*gpio, i2c_write_byte_data(OPR_MODE_ADDR, std::byte{OPERATION_MODE_CONFIG}))
        .Times(1)
        .WillOnce(Return(tl::make_unexpected(interface::GpioError::I2cWriteFailed)));

    auto bno055_model = imu::Bno055Model(std::move(gpio));
    auto result = bno055_model.begin();

    ASSERT_FALSE(result);
}

TEST(Bno055ModelBeginTest, fail_on_trigger_reset) {
    auto _ = InSequence{};  // Ensure calls are made in the expected order

    auto gpio = std::make_unique<mock::Gpio>();

    // Activate I2C bus
    EXPECT_CALL(*gpio, i2c_open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Read chip ID
    EXPECT_CALL(*gpio, i2c_read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<std::byte, interface::GpioError>(std::byte{ID})));
    // Set operation mode to CONFIG
    EXPECT_CALL(*gpio, i2c_write_byte_data(OPR_MODE_ADDR, std::byte{OPERATION_MODE_CONFIG}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Trigger reset -> FAIL!
    EXPECT_CALL(*gpio, i2c_write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x20}))
        .Times(1)
        .WillOnce(Return(tl::make_unexpected(interface::GpioError::I2cWriteFailed)));

    auto bno055_model = imu::Bno055Model(std::move(gpio));
    auto result = bno055_model.begin();

    ASSERT_FALSE(result);
}

TEST(Bno055ModelBeginTest, fail_on_wait_for_reboot) {
    auto _ = InSequence{};  // Ensure calls are made in the expected order

    auto gpio = std::make_unique<mock::Gpio>();

    // Activate I2C bus
    EXPECT_CALL(*gpio, i2c_open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Read chip ID
    EXPECT_CALL(*gpio, i2c_read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<std::byte, interface::GpioError>(std::byte{ID})));
    // Set operation mode to CONFIG
    EXPECT_CALL(*gpio, i2c_write_byte_data(OPR_MODE_ADDR, std::byte{OPERATION_MODE_CONFIG}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Trigger reset
    EXPECT_CALL(*gpio, i2c_write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x20}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Wait for reboot (while the chip ID is read again) -> FAIL!
    EXPECT_CALL(*gpio, i2c_read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillRepeatedly(Return(tl::expected<std::byte, interface::GpioError>(std::byte{ID + 1})));

    auto bno055_model = imu::Bno055Model(std::move(gpio));
    auto result = bno055_model.begin();

    ASSERT_FALSE(result);
}

TEST(Bno055ModelBeginTest, fail_on_set_power_mode_normal) {
    auto _ = InSequence{};  // Ensure calls are made in the expected order

    auto gpio = std::make_unique<mock::Gpio>();

    // Activate I2C bus
    EXPECT_CALL(*gpio, i2c_open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Read chip ID
    EXPECT_CALL(*gpio, i2c_read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<std::byte, interface::GpioError>(std::byte{ID})));
    // Set operation mode to CONFIG
    EXPECT_CALL(*gpio, i2c_write_byte_data(OPR_MODE_ADDR, std::byte{OPERATION_MODE_CONFIG}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Trigger reset
    EXPECT_CALL(*gpio, i2c_write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x20}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Wait for reboot (while the chip ID is read again)
    EXPECT_CALL(*gpio, i2c_read_byte_data(CHIP_ID_ADDR))
        .WillOnce(Return(tl::expected<std::byte, interface::GpioError>{std::byte{ID}}));
    // Set power mode to NORMAL -> FAIL!
    EXPECT_CALL(*gpio, i2c_write_byte_data(PWR_MODE_ADDR, std::byte{POWER_MODE_NORMAL}))
        .Times(1)
        .WillOnce(Return(tl::make_unexpected(interface::GpioError::I2cWriteFailed)));

    auto bno055_model = imu::Bno055Model(std::move(gpio));
    auto result = bno055_model.begin();

    ASSERT_FALSE(result);
}

TEST(Bno055ModelBeginTest, fail_on_set_page_id) {
    auto _ = InSequence{};  // Ensure calls are made in the expected order

    auto gpio = std::make_unique<mock::Gpio>();

    // Activate I2C bus
    EXPECT_CALL(*gpio, i2c_open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Read chip ID
    EXPECT_CALL(*gpio, i2c_read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<std::byte, interface::GpioError>(std::byte{ID})));
    // Set operation mode to CONFIG
    EXPECT_CALL(*gpio, i2c_write_byte_data(OPR_MODE_ADDR, std::byte{OPERATION_MODE_CONFIG}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Trigger reset
    EXPECT_CALL(*gpio, i2c_write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x20}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Wait for reboot (while the chip ID is read again)
    EXPECT_CALL(*gpio, i2c_read_byte_data(CHIP_ID_ADDR))
        .WillOnce(Return(tl::expected<std::byte, interface::GpioError>{std::byte{ID}}));
    // Set power mode to NORMAL
    EXPECT_CALL(*gpio, i2c_write_byte_data(PWR_MODE_ADDR, std::byte{POWER_MODE_NORMAL}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Set page ID to 0 -> FAIL!
    EXPECT_CALL(*gpio, i2c_write_byte_data(PAGE_ID_ADDR, std::byte{0x0}))
        .Times(1)
        .WillOnce(Return(tl::make_unexpected(interface::GpioError::I2cWriteFailed)));

    auto bno055_model = imu::Bno055Model(std::move(gpio));
    auto result = bno055_model.begin();

    ASSERT_FALSE(result);
}

TEST(Bno055ModelBeginTest, fail_on_clear_sys_trigger) {
    auto _ = InSequence{};  // Ensure calls are made in the expected order

    auto gpio = std::make_unique<mock::Gpio>();

    // Activate I2C bus
    EXPECT_CALL(*gpio, i2c_open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Read chip ID
    EXPECT_CALL(*gpio, i2c_read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<std::byte, interface::GpioError>(std::byte{ID})));
    // Set operation mode to CONFIG
    EXPECT_CALL(*gpio, i2c_write_byte_data(OPR_MODE_ADDR, std::byte{OPERATION_MODE_CONFIG}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Trigger reset
    EXPECT_CALL(*gpio, i2c_write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x20}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Wait for reboot (while the chip ID is read again)
    EXPECT_CALL(*gpio, i2c_read_byte_data(CHIP_ID_ADDR))
        .WillOnce(Return(tl::expected<std::byte, interface::GpioError>{std::byte{ID}}));
    // Set power mode to NORMAL
    EXPECT_CALL(*gpio, i2c_write_byte_data(PWR_MODE_ADDR, std::byte{POWER_MODE_NORMAL}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Set page ID to 0
    EXPECT_CALL(*gpio, i2c_write_byte_data(PAGE_ID_ADDR, std::byte{0x0}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Clear SYS_TRIGGER -> FAIL!
    EXPECT_CALL(*gpio, i2c_write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x0}))
        .Times(1)
        .WillOnce(Return(tl::make_unexpected(interface::GpioError::I2cWriteFailed)));

    auto bno055_model = imu::Bno055Model(std::move(gpio));
    auto result = bno055_model.begin();

    ASSERT_FALSE(result);
}

TEST(Bno055ModelBeginTest, fail_on_set_opr_mode_ndof) {
    auto _ = InSequence{};  // Ensure calls are made in the expected order

    auto gpio = std::make_unique<mock::Gpio>();

    // Activate I2C bus
    EXPECT_CALL(*gpio, i2c_open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Read chip ID
    EXPECT_CALL(*gpio, i2c_read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<std::byte, interface::GpioError>(std::byte{ID})));
    // Set operation mode to CONFIG
    EXPECT_CALL(*gpio, i2c_write_byte_data(OPR_MODE_ADDR, std::byte{OPERATION_MODE_CONFIG}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Trigger reset
    EXPECT_CALL(*gpio, i2c_write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x20}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Wait for reboot (while the chip ID is read again)
    EXPECT_CALL(*gpio, i2c_read_byte_data(CHIP_ID_ADDR))
        .WillOnce(Return(tl::expected<std::byte, interface::GpioError>{std::byte{ID}}));
    // Set power mode to NORMAL
    EXPECT_CALL(*gpio, i2c_write_byte_data(PWR_MODE_ADDR, std::byte{POWER_MODE_NORMAL}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Set page ID to 0
    EXPECT_CALL(*gpio, i2c_write_byte_data(PAGE_ID_ADDR, std::byte{0x0}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Clear SYS_TRIGGER
    EXPECT_CALL(*gpio, i2c_write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x0}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    // Set operation mode to NDOF -> FAIL!
    EXPECT_CALL(*gpio, i2c_write_byte_data(OPR_MODE_ADDR, std::byte{OPERATION_MODE_NDOF}))
        .Times(1)
        .WillOnce(Return(tl::make_unexpected(interface::GpioError::I2cWriteFailed)));

    auto bno055_model = imu::Bno055Model(std::move(gpio));
    auto result = bno055_model.begin();

    ASSERT_FALSE(result);
}

TEST(Bno055ModelGetAccelerationTest, success) {
    auto gpio = std::make_unique<mock::Gpio>();

    constexpr std::byte MOCK_DATA[6] = {std::byte{0x00}, std::byte{0x10},
                                        std::byte{0x00}, std::byte{0x20},
                                        std::byte{0x00}, std::byte{0x30}};  // Example data

    // LSB, MSBの順で値を合成
    const auto expected_accel = state::imu::Acceleration{
        static_cast<double>(
            std::to_integer<int16_t>(MOCK_DATA[0]) |
            (std::to_integer<int16_t>(MOCK_DATA[1]) << 8)) *
            ACCEL_SCALE,
        static_cast<double>(
            std::to_integer<int16_t>(MOCK_DATA[2]) |
            (std::to_integer<int16_t>(MOCK_DATA[3]) << 8)) *
            ACCEL_SCALE,
        static_cast<double>(
            std::to_integer<int16_t>(MOCK_DATA[4]) |
            (std::to_integer<int16_t>(MOCK_DATA[5]) << 8)) *
            ACCEL_SCALE};

    EXPECT_CALL(*gpio, i2c_read_block_data(ACCEL_ADDR, _, _))
        .Times(1)
        .WillOnce(DoAll(
            testing::SetArrayArgument<1>(MOCK_DATA, MOCK_DATA + 6),
            Return(tl::expected<void, interface::GpioError>{})));

    auto bno055_model = imu::Bno055Model{std::move(gpio)};
    const auto result = bno055_model.get_acceleration();

    ASSERT_TRUE(result);
    EXPECT_EQ(result.value().x, expected_accel.x);
    EXPECT_EQ(result.value().y, expected_accel.y);
    EXPECT_EQ(result.value().z, expected_accel.z);
}

TEST(Bno055ModelGetAccelerationTest, fail_on_get_vector) {
    auto gpio = std::make_unique<mock::Gpio>();

    EXPECT_CALL(*gpio, i2c_read_block_data(ACCEL_ADDR, testing::NotNull(), 6))
        .Times(1)
        .WillOnce(Return(tl::make_unexpected(interface::GpioError::I2cReadFailed)));

    auto bno055_model = imu::Bno055Model{std::move(gpio)};
    const auto result = bno055_model.get_acceleration();

    ASSERT_FALSE(result);
}

TEST(Bno055ModelGetAngularVelocity, success) {
    auto gpio = std::make_unique<mock::Gpio>();

    constexpr std::byte MOCK_DATA[6] = {std::byte{0x00}, std::byte{0x10},
                                        std::byte{0x00}, std::byte{0x20},
                                        std::byte{0x00}, std::byte{0x30}};  // Example data

    EXPECT_CALL(*gpio, i2c_read_block_data(ANGVEL_ADDR, _, _))
        .Times(1)
        .WillOnce(DoAll(
            testing::SetArrayArgument<1>(MOCK_DATA, MOCK_DATA + 6),
            Return(tl::expected<void, interface::GpioError>{})));

    const auto expected_angular_vel = state::imu::AngularVelocity{
        static_cast<double>(
            std::to_integer<int16_t>(MOCK_DATA[0]) |
            (std::to_integer<int16_t>(MOCK_DATA[1]) << 8)) *
            ANGVEL_SCALE,
        static_cast<double>(
            std::to_integer<int16_t>(MOCK_DATA[2]) |
            (std::to_integer<int16_t>(MOCK_DATA[3]) << 8)) *
            ANGVEL_SCALE,
        static_cast<double>(
            std::to_integer<int16_t>(MOCK_DATA[4]) |
            (std::to_integer<int16_t>(MOCK_DATA[5]) << 8)) *
            ANGVEL_SCALE};

    auto bno055_model = imu::Bno055Model{std::move(gpio)};
    const auto result = bno055_model.get_angular_velocity();

    ASSERT_TRUE(result);
    EXPECT_EQ(result.value().x, expected_angular_vel.x);
    EXPECT_EQ(result.value().y, expected_angular_vel.y);
    EXPECT_EQ(result.value().z, expected_angular_vel.z);
}

// TODO: Add success cases for get_quat and get_temp
TEST(Bno055ModelGetTempTest, success) {
    auto gpio = std::make_unique<mock::Gpio>();

    constexpr auto MOCK_DATA = std::byte{0x12};
    const auto expected_temp =
        state::imu::Temperature{std::to_integer<int8_t>(MOCK_DATA & std::byte{0x7F})};

    EXPECT_CALL(*gpio, i2c_read_byte_data(TEMP_ADDR))
        .Times(1)
        .WillOnce(Return(tl::expected<std::byte, interface::GpioError>{MOCK_DATA}));

    auto bno055_model = imu::Bno055Model{std::move(gpio)};
    const auto result = bno055_model.get_temp();

    ASSERT_TRUE(result);
    ASSERT_EQ(result.value().value, expected_temp.value);
}

TEST(Bno055ModelGetTempTest, fail_on_get_temperature) {
    auto gpio = std::make_unique<mock::Gpio>();

    EXPECT_CALL(*gpio, i2c_read_byte_data(TEMP_ADDR))
        .Times(1)
        .WillOnce(Return(tl::make_unexpected(interface::GpioError::I2cReadFailed)));

    auto bno055_model = imu::Bno055Model(std::move(gpio));
    auto result = bno055_model.get_temp();

    ASSERT_FALSE(result);
}

TEST(Bno055ModelGetQuatTest, success) {
    auto gpio = std::make_unique<mock::Gpio>();

    constexpr std::byte MOCK_DATA[8] = {std::byte{0x00}, std::byte{0x10}, std::byte{0x00},
                                        std::byte{0x20}, std::byte{0x00}, std::byte{0x30},
                                        std::byte{0x00}, std::byte{0x40}};  // Example data

    // LSB, MSBの順で値を合成
    const auto expected_quat = state::imu::Quaternion{
        static_cast<double>(
            std::to_integer<int16_t>(MOCK_DATA[0]) |
            (std::to_integer<int16_t>(MOCK_DATA[1]) << 8)) *
            QUAT_SCALE,
        static_cast<double>(
            std::to_integer<int16_t>(MOCK_DATA[2]) |
            (std::to_integer<int16_t>(MOCK_DATA[3]) << 8)) *
            QUAT_SCALE,
        static_cast<double>(
            std::to_integer<int16_t>(MOCK_DATA[4]) |
            (std::to_integer<int16_t>(MOCK_DATA[5]) << 8)) *
            QUAT_SCALE,
        static_cast<double>(
            std::to_integer<int16_t>(MOCK_DATA[6]) |
            (std::to_integer<int16_t>(MOCK_DATA[7]) << 8)) *
            QUAT_SCALE};

    EXPECT_CALL(*gpio, i2c_read_block_data(QUAT_ADDR, _, _))
        .Times(1)
        .WillOnce(DoAll(
            testing::SetArrayArgument<1>(MOCK_DATA, MOCK_DATA + 8),
            Return(tl::expected<void, interface::GpioError>{})));

    auto bno055_model = imu::Bno055Model{std::move(gpio)};
    const auto result = bno055_model.get_quat();

    ASSERT_TRUE(result);
    EXPECT_EQ(result.value().x, expected_quat.x);
    EXPECT_EQ(result.value().y, expected_quat.y);
    EXPECT_EQ(result.value().z, expected_quat.z);
    EXPECT_EQ(result.value().w, expected_quat.w);
}

TEST(Bno055ModelGetQuatTest, fail_on_get_quaternion) {
    auto gpio = std::make_unique<mock::Gpio>();

    EXPECT_CALL(*gpio, i2c_read_block_data(QUAT_ADDR, _, _))
        .Times(1)
        .WillOnce(Return(tl::make_unexpected(interface::GpioError::I2cReadFailed)));

    auto bno055_model = imu::Bno055Model(std::move(gpio));
    auto result = bno055_model.get_quat();

    ASSERT_FALSE(result);
}

}  // namespace sinsei_umiusi_control::test::hardware_model::bno055

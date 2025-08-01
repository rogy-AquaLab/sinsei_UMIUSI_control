#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <rcpputils/tl_expected/expected.hpp>

#include "mock/gpio.hpp"
#include "sinsei_umiusi_control/hardware_model/imu_model.hpp"

using namespace sinsei_umiusi_control::hardware_model;

using testing::AtLeast;
using testing::InSequence;
using testing::Return;

namespace sinsei_umiusi_control::test::hardware_model::imu {

static constexpr auto ADDRESS = ::sinsei_umiusi_control::hardware_model::ImuModel::ADDRESS;
static constexpr auto ID = ::sinsei_umiusi_control::hardware_model::ImuModel::ID;
static constexpr auto CHIP_ID_ADDR =
    ::sinsei_umiusi_control::hardware_model::ImuModel::CHIP_ID_ADDR;
static constexpr auto OPR_MODE_ADDR =
    ::sinsei_umiusi_control::hardware_model::ImuModel::OPR_MODE_ADDR;
static constexpr auto SYS_TRIGGER_ADDR =
    ::sinsei_umiusi_control::hardware_model::ImuModel::SYS_TRIGGER_ADDR;
static constexpr auto PWR_MODE_ADDR =
    ::sinsei_umiusi_control::hardware_model::ImuModel::PWR_MODE_ADDR;
static constexpr auto PAGE_ID_ADDR =
    ::sinsei_umiusi_control::hardware_model::ImuModel::PAGE_ID_ADDR;
static constexpr auto QUATERNION_DATA_W_LSB_ADDR =
    ::sinsei_umiusi_control::hardware_model::ImuModel::QUATERNION_DATA_W_LSB_ADDR;
static constexpr auto OPERATION_MODE_CONFIG =
    ::sinsei_umiusi_control::hardware_model::ImuModel::OPERATION_MODE_CONFIG;
static constexpr auto OPERATION_MODE_NDOF =
    ::sinsei_umiusi_control::hardware_model::ImuModel::OPERATION_MODE_NDOF;
static constexpr auto POWER_MODE_NORMAL =
    ::sinsei_umiusi_control::hardware_model::ImuModel::POWER_MODE_NORMAL;
static constexpr auto TEMP_ADDR = ::sinsei_umiusi_control::hardware_model::ImuModel::TEMP_ADDR;

TEST(ImuModelBeginTest, success) {
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

    auto imu_model = ImuModel(std::move(gpio));
    auto result = imu_model.on_init();

    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
}

TEST(ImuModelBeginTest, fail_on_open) {
    auto gpio = std::make_unique<mock::Gpio>();

    // Activate I2C bus -> FAIL!
    EXPECT_CALL(*gpio, i2c_open(ADDRESS))
        .Times(AtLeast(1))
        .WillRepeatedly(Return(tl::make_unexpected(interface::GpioError::I2cOpenFailed)));

    auto imu_model = ImuModel(std::move(gpio));
    auto result = imu_model.on_init();

    ASSERT_FALSE(result);
}

TEST(ImuModelBeginTest, fail_on_read_chip_id) {
    auto _ = InSequence{};  // Ensure calls are made in the expected order

    auto gpio = std::make_unique<mock::Gpio>();

    // Activate I2C bus
    EXPECT_CALL(*gpio, i2c_open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, interface::GpioError>{}));
    EXPECT_CALL(*gpio, i2c_read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::make_unexpected(interface::GpioError::I2cReadFailed)));

    auto imu_model = ImuModel(std::move(gpio));
    auto result = imu_model.on_init();

    ASSERT_FALSE(result);
}

TEST(ImuModelBeginTest, fail_on_wrong_chip_id) {
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

    auto imu_model = ImuModel(std::move(gpio));
    auto result = imu_model.on_init();

    ASSERT_FALSE(result);
}

TEST(ImuModelBeginTest, fail_on_set_opr_mode_config) {
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

    auto imu_model = ImuModel(std::move(gpio));
    auto result = imu_model.on_init();

    ASSERT_FALSE(result);
}

TEST(ImuModelBeginTest, fail_on_trigger_reset) {
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

    auto imu_model = ImuModel(std::move(gpio));
    auto result = imu_model.on_init();

    ASSERT_FALSE(result);
}

TEST(ImuModelBeginTest, fail_on_wait_for_reboot) {
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

    auto imu_model = ImuModel(std::move(gpio));
    auto result = imu_model.on_init();

    ASSERT_FALSE(result);
}

TEST(ImuModelBeginTest, fail_on_set_power_mode_normal) {
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

    auto imu_model = ImuModel(std::move(gpio));
    auto result = imu_model.on_init();

    ASSERT_FALSE(result);
}

TEST(ImuModelBeginTest, fail_on_set_page_id) {
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

    auto imu_model = ImuModel(std::move(gpio));
    auto result = imu_model.on_init();

    ASSERT_FALSE(result);
}

TEST(ImuModelBeginTest, fail_on_clear_sys_trigger) {
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

    auto imu_model = ImuModel(std::move(gpio));
    auto result = imu_model.on_init();

    ASSERT_FALSE(result);
}

TEST(ImuModelBeginTest, fail_on_set_opr_mode_ndof) {
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

    auto imu_model = ImuModel(std::move(gpio));
    auto result = imu_model.on_init();

    ASSERT_FALSE(result);
}

TEST(ImuModelOnReadTest, all) {
    auto gpio = std::make_unique<mock::Gpio>();

    auto dummy_quat = std::array<int16_t, 8>{};  // Dummy data for quaternion
    for (int i = 0; i < 8; ++i) {
        dummy_quat[i] = static_cast<int16_t>(i);  // dummy

        EXPECT_CALL(*gpio, i2c_read_byte_data(QUATERNION_DATA_W_LSB_ADDR + i))
            .Times(1)
            .WillOnce(Return(tl::expected<std::byte, interface::GpioError>(
                static_cast<std::byte>(dummy_quat[i]))));
    }
    const auto dummy_temp = std::byte{0x1A};  // Dummy data for temperature
    EXPECT_CALL(*gpio, i2c_read_byte_data(TEMP_ADDR))
        .Times(1)
        .WillOnce(Return(tl::expected<std::byte, interface::GpioError>(dummy_temp)));

    auto imu_model = ImuModel(std::move(gpio));
    auto result = imu_model.on_read();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();

    // FIXME: 速度は未実装
    const auto & [quaternion, _velocity, temperature] = result.value();

    constexpr double SCALE = 1.0 / (1 << 14);
    const auto expected_w = static_cast<double>(dummy_quat[0] | dummy_quat[1] << 8) * SCALE;
    const auto expected_x = static_cast<double>(dummy_quat[2] | dummy_quat[3] << 8) * SCALE;
    const auto expected_y = static_cast<double>(dummy_quat[4] | dummy_quat[5] << 8) * SCALE;
    const auto expected_z = static_cast<double>(dummy_quat[6] | dummy_quat[7] << 8) * SCALE;
    const auto expected_temp = static_cast<int8_t>(dummy_temp);
    EXPECT_DOUBLE_EQ(quaternion.w, expected_w);
    EXPECT_DOUBLE_EQ(quaternion.x, expected_x);
    EXPECT_DOUBLE_EQ(quaternion.y, expected_y);
    EXPECT_DOUBLE_EQ(quaternion.z, expected_z);
    EXPECT_EQ(temperature.value, expected_temp);
}

TEST(ImuModelOnReadTest, fail_on_read_quaternion) {
    auto gpio = std::make_unique<mock::Gpio>();

    EXPECT_CALL(*gpio, i2c_read_byte_data(QUATERNION_DATA_W_LSB_ADDR))
        .Times(1)
        .WillOnce(Return(tl::make_unexpected(interface::GpioError::I2cReadFailed)));

    auto imu_model = ImuModel(std::move(gpio));
    auto result = imu_model.on_read();

    ASSERT_FALSE(result);
}

TEST(ImuModelOnReadTest, fail_on_read_temperature) {
    auto gpio = std::make_unique<mock::Gpio>();

    for (int i = 0; i < 8; ++i) {
        EXPECT_CALL(*gpio, i2c_read_byte_data(QUATERNION_DATA_W_LSB_ADDR + i))
            .WillOnce(Return(tl::expected<std::byte, interface::GpioError>(
                std::byte{0x00})));  // Dummy data for quaternion
    }

    EXPECT_CALL(*gpio, i2c_read_byte_data(TEMP_ADDR))
        .Times(1)
        .WillOnce(Return(tl::make_unexpected(interface::GpioError::I2cReadFailed)));

    auto imu_model = ImuModel(std::move(gpio));
    auto result = imu_model.on_read();

    ASSERT_FALSE(result);
}

}  // namespace sinsei_umiusi_control::test::hardware_model::imu

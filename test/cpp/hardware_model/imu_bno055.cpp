#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>

#include "mock/i2c.hpp"
#include "sinsei_umiusi_control/hardware_model/imu_model.hpp"
#include "sinsei_umiusi_control/hardware_model/interface/i2c.hpp"

using namespace sinsei_umiusi_control::hardware_model;

using ::testing::AtLeast;
using ::testing::InSequence;
using ::testing::Return;

namespace {

constexpr auto _ = ::testing::_;

}

namespace sinsei_umiusi_control::test::hardware_model::imu_bno055 {

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
static constexpr auto OPERATION_MODE_CONFIG =
    ::sinsei_umiusi_control::hardware_model::ImuModel::OPERATION_MODE_CONFIG;
static constexpr auto OPERATION_MODE_NDOF =
    ::sinsei_umiusi_control::hardware_model::ImuModel::OPERATION_MODE_NDOF;
static constexpr auto POWER_MODE_NORMAL =
    ::sinsei_umiusi_control::hardware_model::ImuModel::POWER_MODE_NORMAL;
static constexpr auto ACCEL_ADDR =
    ::sinsei_umiusi_control::hardware_model::ImuModel::LINEAR_ACCEL_DATA_X_LSB_ADDR;
static constexpr auto ANGVEL_ADDR =
    ::sinsei_umiusi_control::hardware_model::ImuModel::GYRO_DATA_X_LSB_ADDR;
static constexpr auto TEMP_ADDR = ::sinsei_umiusi_control::hardware_model::ImuModel::TEMP_ADDR;
static constexpr auto QUAT_ADDR =
    ::sinsei_umiusi_control::hardware_model::ImuModel::QUATERNION_DATA_W_LSB_ADDR;
static constexpr auto ACCEL_SCALE = 1.0 / 100.0;
static constexpr auto ANGVEL_SCALE = 1.0 / 16.0;
static constexpr auto QUAT_SCALE = 1.0 / (1 << 14);

TEST(ImuModelBeginTest, success) {
    auto _ = InSequence{};  // Ensure calls are made in the expected order

    auto gpio = std::make_unique<mock::I2c>();

    // Activate I2C bus
    EXPECT_CALL(*gpio, open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Read chip ID
    EXPECT_CALL(*gpio, read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<std::byte, interface::I2cError>(std::byte{ID})));
    // Set operation mode to CONFIG
    EXPECT_CALL(*gpio, write_byte_data(OPR_MODE_ADDR, std::byte{OPERATION_MODE_CONFIG}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Trigger reset
    EXPECT_CALL(*gpio, write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x20}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Wait for reboot (while the chip ID is read again)
    EXPECT_CALL(*gpio, read_byte_data(CHIP_ID_ADDR))
        .WillOnce(Return(tl::expected<std::byte, interface::I2cError>{std::byte{ID}}));
    // Set power mode to NORMAL
    EXPECT_CALL(*gpio, write_byte_data(PWR_MODE_ADDR, std::byte{POWER_MODE_NORMAL}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Set page ID to 0
    EXPECT_CALL(*gpio, write_byte_data(PAGE_ID_ADDR, std::byte{0x0}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Clear SYS_TRIGGER
    EXPECT_CALL(*gpio, write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x0}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Set operation mode to NDOF
    EXPECT_CALL(*gpio, write_byte_data(OPR_MODE_ADDR, std::byte{OPERATION_MODE_NDOF}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));

    auto imu_model = ImuModel(std::move(gpio));
    auto result = imu_model.on_init();

    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
}

TEST(ImuModelBeginTest, fail_on_open) {
    auto gpio = std::make_unique<mock::I2c>();

    // Activate I2C bus -> FAIL!
    EXPECT_CALL(*gpio, open(ADDRESS))
        .Times(AtLeast(1))
        .WillRepeatedly(Return(tl::make_unexpected(interface::I2cError::OpenFailed)));

    auto imu_model = ImuModel(std::move(gpio));
    auto result = imu_model.on_init();

    ASSERT_FALSE(result);
}

TEST(ImuModelBeginTest, fail_on_read_chip_id) {
    auto _ = InSequence{};  // Ensure calls are made in the expected order

    auto gpio = std::make_unique<mock::I2c>();

    // Activate I2C bus
    EXPECT_CALL(*gpio, open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    EXPECT_CALL(*gpio, read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::make_unexpected(interface::I2cError::ReadFailed)));

    auto imu_model = ImuModel(std::move(gpio));
    auto result = imu_model.on_init();

    ASSERT_FALSE(result);
}

TEST(ImuModelBeginTest, fail_on_wrong_chip_id) {
    auto _ = InSequence{};  // Ensure calls are made in the expected order

    auto gpio = std::make_unique<mock::I2c>();

    // Activate I2C bus
    EXPECT_CALL(*gpio, open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Read chip ID -> FAIL!
    EXPECT_CALL(*gpio, read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<std::byte, interface::I2cError>(std::byte{ID + 1})));

    auto imu_model = ImuModel(std::move(gpio));
    auto result = imu_model.on_init();

    ASSERT_FALSE(result);
}

TEST(ImuModelBeginTest, fail_on_set_opr_mode_config) {
    auto _ = InSequence{};  // Ensure calls are made in the expected order

    auto gpio = std::make_unique<mock::I2c>();

    // Activate I2C bus
    EXPECT_CALL(*gpio, open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Read chip ID
    EXPECT_CALL(*gpio, read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<std::byte, interface::I2cError>(std::byte{ID})));
    // Set operation mode to CONFIG -> FAIL!
    EXPECT_CALL(*gpio, write_byte_data(OPR_MODE_ADDR, std::byte{OPERATION_MODE_CONFIG}))
        .Times(1)
        .WillOnce(Return(tl::make_unexpected(interface::I2cError::WriteFailed)));

    auto imu_model = ImuModel(std::move(gpio));
    auto result = imu_model.on_init();

    ASSERT_FALSE(result);
}

TEST(ImuModelBeginTest, fail_on_trigger_reset) {
    auto _ = InSequence{};  // Ensure calls are made in the expected order

    auto gpio = std::make_unique<mock::I2c>();

    // Activate I2C bus
    EXPECT_CALL(*gpio, open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Read chip ID
    EXPECT_CALL(*gpio, read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<std::byte, interface::I2cError>(std::byte{ID})));
    // Set operation mode to CONFIG
    EXPECT_CALL(*gpio, write_byte_data(OPR_MODE_ADDR, std::byte{OPERATION_MODE_CONFIG}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Trigger reset -> FAIL!
    EXPECT_CALL(*gpio, write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x20}))
        .Times(1)
        .WillOnce(Return(tl::make_unexpected(interface::I2cError::WriteFailed)));

    auto imu_model = ImuModel(std::move(gpio));
    auto result = imu_model.on_init();

    ASSERT_FALSE(result);
}

TEST(ImuModelBeginTest, fail_on_wait_for_reboot) {
    auto _ = InSequence{};  // Ensure calls are made in the expected order

    auto gpio = std::make_unique<mock::I2c>();

    // Activate I2C bus
    EXPECT_CALL(*gpio, open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Read chip ID
    EXPECT_CALL(*gpio, read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<std::byte, interface::I2cError>(std::byte{ID})));
    // Set operation mode to CONFIG
    EXPECT_CALL(*gpio, write_byte_data(OPR_MODE_ADDR, std::byte{OPERATION_MODE_CONFIG}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Trigger reset
    EXPECT_CALL(*gpio, write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x20}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Wait for reboot (while the chip ID is read again) -> FAIL!
    EXPECT_CALL(*gpio, read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillRepeatedly(Return(tl::expected<std::byte, interface::I2cError>(std::byte{ID + 1})));

    auto imu_model = ImuModel(std::move(gpio));
    auto result = imu_model.on_init();

    ASSERT_FALSE(result);
}

TEST(ImuModelBeginTest, fail_on_set_power_mode_normal) {
    auto _ = InSequence{};  // Ensure calls are made in the expected order

    auto gpio = std::make_unique<mock::I2c>();

    // Activate I2C bus
    EXPECT_CALL(*gpio, open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Read chip ID
    EXPECT_CALL(*gpio, read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<std::byte, interface::I2cError>(std::byte{ID})));
    // Set operation mode to CONFIG
    EXPECT_CALL(*gpio, write_byte_data(OPR_MODE_ADDR, std::byte{OPERATION_MODE_CONFIG}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Trigger reset
    EXPECT_CALL(*gpio, write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x20}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Wait for reboot (while the chip ID is read again)
    EXPECT_CALL(*gpio, read_byte_data(CHIP_ID_ADDR))
        .WillOnce(Return(tl::expected<std::byte, interface::I2cError>{std::byte{ID}}));
    // Set power mode to NORMAL -> FAIL!
    EXPECT_CALL(*gpio, write_byte_data(PWR_MODE_ADDR, std::byte{POWER_MODE_NORMAL}))
        .Times(1)
        .WillOnce(Return(tl::make_unexpected(interface::I2cError::WriteFailed)));

    auto imu_model = ImuModel(std::move(gpio));
    auto result = imu_model.on_init();

    ASSERT_FALSE(result);
}

TEST(ImuModelBeginTest, fail_on_set_page_id) {
    auto _ = InSequence{};  // Ensure calls are made in the expected order

    auto gpio = std::make_unique<mock::I2c>();

    // Activate I2C bus
    EXPECT_CALL(*gpio, open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Read chip ID
    EXPECT_CALL(*gpio, read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<std::byte, interface::I2cError>(std::byte{ID})));
    // Set operation mode to CONFIG
    EXPECT_CALL(*gpio, write_byte_data(OPR_MODE_ADDR, std::byte{OPERATION_MODE_CONFIG}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Trigger reset
    EXPECT_CALL(*gpio, write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x20}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Wait for reboot (while the chip ID is read again)
    EXPECT_CALL(*gpio, read_byte_data(CHIP_ID_ADDR))
        .WillOnce(Return(tl::expected<std::byte, interface::I2cError>{std::byte{ID}}));
    // Set power mode to NORMAL
    EXPECT_CALL(*gpio, write_byte_data(PWR_MODE_ADDR, std::byte{POWER_MODE_NORMAL}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Set page ID to 0 -> FAIL!
    EXPECT_CALL(*gpio, write_byte_data(PAGE_ID_ADDR, std::byte{0x0}))
        .Times(1)
        .WillOnce(Return(tl::make_unexpected(interface::I2cError::WriteFailed)));

    auto imu_model = ImuModel(std::move(gpio));
    auto result = imu_model.on_init();

    ASSERT_FALSE(result);
}

TEST(ImuModelBeginTest, fail_on_clear_sys_trigger) {
    auto _ = InSequence{};  // Ensure calls are made in the expected order

    auto gpio = std::make_unique<mock::I2c>();

    // Activate I2C bus
    EXPECT_CALL(*gpio, open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Read chip ID
    EXPECT_CALL(*gpio, read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<std::byte, interface::I2cError>(std::byte{ID})));
    // Set operation mode to CONFIG
    EXPECT_CALL(*gpio, write_byte_data(OPR_MODE_ADDR, std::byte{OPERATION_MODE_CONFIG}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Trigger reset
    EXPECT_CALL(*gpio, write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x20}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Wait for reboot (while the chip ID is read again)
    EXPECT_CALL(*gpio, read_byte_data(CHIP_ID_ADDR))
        .WillOnce(Return(tl::expected<std::byte, interface::I2cError>{std::byte{ID}}));
    // Set power mode to NORMAL
    EXPECT_CALL(*gpio, write_byte_data(PWR_MODE_ADDR, std::byte{POWER_MODE_NORMAL}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Set page ID to 0
    EXPECT_CALL(*gpio, write_byte_data(PAGE_ID_ADDR, std::byte{0x0}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Clear SYS_TRIGGER -> FAIL!
    EXPECT_CALL(*gpio, write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x0}))
        .Times(1)
        .WillOnce(Return(tl::make_unexpected(interface::I2cError::WriteFailed)));

    auto imu_model = ImuModel(std::move(gpio));
    auto result = imu_model.on_init();

    ASSERT_FALSE(result);
}

TEST(ImuModelBeginTest, fail_on_set_opr_mode_ndof) {
    auto _ = InSequence{};  // Ensure calls are made in the expected order

    auto gpio = std::make_unique<mock::I2c>();

    // Activate I2C bus
    EXPECT_CALL(*gpio, open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Read chip ID
    EXPECT_CALL(*gpio, read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<std::byte, interface::I2cError>(std::byte{ID})));
    // Set operation mode to CONFIG
    EXPECT_CALL(*gpio, write_byte_data(OPR_MODE_ADDR, std::byte{OPERATION_MODE_CONFIG}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Trigger reset
    EXPECT_CALL(*gpio, write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x20}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Wait for reboot (while the chip ID is read again)
    EXPECT_CALL(*gpio, read_byte_data(CHIP_ID_ADDR))
        .WillOnce(Return(tl::expected<std::byte, interface::I2cError>{std::byte{ID}}));
    // Set power mode to NORMAL
    EXPECT_CALL(*gpio, write_byte_data(PWR_MODE_ADDR, std::byte{POWER_MODE_NORMAL}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Set page ID to 0
    EXPECT_CALL(*gpio, write_byte_data(PAGE_ID_ADDR, std::byte{0x0}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Clear SYS_TRIGGER
    EXPECT_CALL(*gpio, write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x0}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, interface::I2cError>{}));
    // Set operation mode to NDOF -> FAIL!
    EXPECT_CALL(*gpio, write_byte_data(OPR_MODE_ADDR, std::byte{OPERATION_MODE_NDOF}))
        .Times(1)
        .WillOnce(Return(tl::make_unexpected(interface::I2cError::WriteFailed)));

    auto imu_model = ImuModel(std::move(gpio));
    auto result = imu_model.on_init();

    ASSERT_FALSE(result);
}

TEST(ImuModelReadMeasurementsTest, success) {
    auto gpio = std::make_unique<mock::I2c>();

    static constexpr size_t FRAME_LENGTH = TEMP_ADDR - ANGVEL_ADDR + 1;
    auto mock_data = std::array<std::byte, FRAME_LENGTH>{};

    const auto write_s16 = [&mock_data](size_t offset, int16_t value) {
        const auto raw = static_cast<uint16_t>(value);
        mock_data[offset] = std::byte{static_cast<uint8_t>(raw & 0xFF)};
        mock_data[offset + 1] = std::byte{static_cast<uint8_t>((raw >> 8) & 0xFF)};
    };

    constexpr auto OFFSET_QUAT = static_cast<size_t>(QUAT_ADDR - ANGVEL_ADDR);
    constexpr auto OFFSET_LINEAR_ACCEL = static_cast<size_t>(ACCEL_ADDR - ANGVEL_ADDR);
    constexpr auto OFFSET_TEMP = static_cast<size_t>(TEMP_ADDR - ANGVEL_ADDR);

    constexpr int16_t GYRO_X_RAW = 0x0100;
    constexpr int16_t GYRO_Y_RAW = 0x0200;
    constexpr int16_t GYRO_Z_RAW = -0x0100;
    write_s16(0, GYRO_X_RAW);
    write_s16(2, GYRO_Y_RAW);
    write_s16(4, GYRO_Z_RAW);

    constexpr int16_t QUAT_W_RAW = 0x1000;
    constexpr int16_t QUAT_X_RAW = 0x0800;
    constexpr int16_t QUAT_Y_RAW = -0x0400;
    constexpr int16_t QUAT_Z_RAW = 0x0200;
    write_s16(OFFSET_QUAT + 0, QUAT_W_RAW);
    write_s16(OFFSET_QUAT + 2, QUAT_X_RAW);
    write_s16(OFFSET_QUAT + 4, QUAT_Y_RAW);
    write_s16(OFFSET_QUAT + 6, QUAT_Z_RAW);

    constexpr int16_t ACCEL_X_RAW = 0x0064;
    constexpr int16_t ACCEL_Y_RAW = -0x00C8;
    constexpr int16_t ACCEL_Z_RAW = 0x012C;
    write_s16(OFFSET_LINEAR_ACCEL + 0, ACCEL_X_RAW);
    write_s16(OFFSET_LINEAR_ACCEL + 2, ACCEL_Y_RAW);
    write_s16(OFFSET_LINEAR_ACCEL + 4, ACCEL_Z_RAW);

    constexpr auto TEMP_RAW = std::byte{0x82};
    mock_data[OFFSET_TEMP] = TEMP_RAW;

    EXPECT_CALL(*gpio, transfer(_))
        .Times(1)
        .WillOnce(testing::Invoke([&mock_data](const std::vector<interface::I2cMessage> & msgs) {
            EXPECT_EQ(msgs.size(), 2U);
            EXPECT_EQ(msgs[0].direction, interface::I2cDirection::Write);
            EXPECT_EQ(msgs[0].length, 1U);
            EXPECT_EQ(msgs[1].direction, interface::I2cDirection::Read);
            EXPECT_EQ(msgs[1].length, mock_data.size());
            std::memcpy(msgs[1].data, mock_data.data(), mock_data.size());
            return tl::expected<void, interface::I2cError>{};
        }));

    auto imu_model = ImuModel{std::move(gpio)};
    const auto result = imu_model.on_read();

    ASSERT_TRUE(result);
    const auto [quaternion, acceleration, angular_velocity, temperature] = result.value();

    EXPECT_DOUBLE_EQ(angular_velocity.x, static_cast<double>(GYRO_X_RAW) * ANGVEL_SCALE);
    EXPECT_DOUBLE_EQ(angular_velocity.y, static_cast<double>(GYRO_Y_RAW) * ANGVEL_SCALE);
    EXPECT_DOUBLE_EQ(angular_velocity.z, static_cast<double>(GYRO_Z_RAW) * ANGVEL_SCALE);

    EXPECT_DOUBLE_EQ(quaternion.x, static_cast<double>(QUAT_W_RAW) * QUAT_SCALE);
    EXPECT_DOUBLE_EQ(quaternion.y, static_cast<double>(QUAT_X_RAW) * QUAT_SCALE);
    EXPECT_DOUBLE_EQ(quaternion.z, static_cast<double>(QUAT_Y_RAW) * QUAT_SCALE);
    EXPECT_DOUBLE_EQ(quaternion.w, static_cast<double>(QUAT_Z_RAW) * QUAT_SCALE);

    EXPECT_DOUBLE_EQ(acceleration.x, static_cast<double>(ACCEL_X_RAW) * ACCEL_SCALE);
    EXPECT_DOUBLE_EQ(acceleration.y, static_cast<double>(ACCEL_Y_RAW) * ACCEL_SCALE);
    EXPECT_DOUBLE_EQ(acceleration.z, static_cast<double>(ACCEL_Z_RAW) * ACCEL_SCALE);

    const auto expected_temp =
        static_cast<int8_t>(std::to_integer<uint8_t>(TEMP_RAW & std::byte{0x7F}));
    EXPECT_EQ(temperature.value, expected_temp);
}

TEST(ImuModelReadMeasurementsTest, fail_on_transfer) {
    auto gpio = std::make_unique<mock::I2c>();

    EXPECT_CALL(*gpio, transfer(_))
        .Times(1)
        .WillOnce(Return(tl::make_unexpected(interface::I2cError::ReadFailed)));

    auto imu_model = ImuModel(std::move(gpio));
    auto result = imu_model.on_read();

    ASSERT_FALSE(result);
}

}  // namespace sinsei_umiusi_control::test::hardware_model::imu_bno055

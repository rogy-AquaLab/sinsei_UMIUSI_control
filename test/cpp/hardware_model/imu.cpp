#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <rcpputils/tl_expected/expected.hpp>

#include "mock/gpio.hpp"
#include "sinsei_umiusi_control/hardware_model/imu_model.hpp"

namespace suchm = sinsei_umiusi_control::hardware_model;
namespace sucutil = sinsei_umiusi_control::util;

using testing::AtLeast;
using testing::Return;

namespace sinsei_umiusi_control::test::hardware_model::imu {

// copied from sinsei_umiusi_control/hardware_model/imu_model.hpp
static constexpr uint32_t ADDRESS = 0x28;
static constexpr uint32_t ID = 0xA0;
static constexpr uint32_t CHIP_ID_ADDR = 0x00;
static constexpr uint32_t OPR_MODE_ADDR = 0x3D;
static constexpr uint32_t SYS_TRIGGER_ADDR = 0x3F;
static constexpr uint32_t PWR_MODE_ADDR = 0x3E;
static constexpr uint32_t PAGE_ID_ADDR = 0x07;
static constexpr uint32_t EULER_H_LSB_ADDR = 0x1A;
static constexpr std::byte OPERATION_MODE_CONFIG{0x00};
static constexpr std::byte OPERATION_MODE_NDOF{0X0C};
static constexpr std::byte POWER_MODE_NORMAL{0x00};
static constexpr uint32_t TEMP_ADDR = 0X34;

TEST(ImuModelBeginTest, success) {
    auto gpio = std::make_unique<mock::Gpio>();
    // TODO: 順序付けする
    EXPECT_CALL(*gpio, i2c_open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillRepeatedly(Return(tl::expected<std::byte, sucutil::GpioError>(std::byte{ID})));
    EXPECT_CALL(*gpio, i2c_write_byte_data(OPR_MODE_ADDR, OPERATION_MODE_CONFIG))
        .Times(1)
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x20}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_write_byte_data(PWR_MODE_ADDR, POWER_MODE_NORMAL))
        .Times(1)
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_write_byte_data(PAGE_ID_ADDR, std::byte{0x0}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x0}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_write_byte_data(OPR_MODE_ADDR, OPERATION_MODE_NDOF))
        .Times(1)
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));

    auto imu_model = suchm::ImuModel(std::move(gpio));
    auto result = imu_model.begin();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
}

TEST(ImuModelBeginTest, fail_on_open) {
    auto gpio = std::make_unique<mock::Gpio>();
    EXPECT_CALL(*gpio, i2c_open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::make_unexpected(sucutil::GpioError::I2cOpenFailed)));

    auto imu_model = suchm::ImuModel(std::move(gpio));
    auto result = imu_model.begin();
    ASSERT_FALSE(result);
}

TEST(ImuModelBeginTest, fail_on_read_chip_id) {
    auto gpio = std::make_unique<mock::Gpio>();
    // TODO: 順序付けする
    EXPECT_CALL(*gpio, i2c_open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillRepeatedly(Return(tl::make_unexpected(sucutil::GpioError::I2cReadFailed)));

    auto imu_model = suchm::ImuModel(std::move(gpio));
    auto result = imu_model.begin();
    ASSERT_FALSE(result);
}

TEST(ImuModelBeginTest, fail_on_wrong_chip_id) {
    // TODO: 順序付けする
    auto gpio = std::make_unique<mock::Gpio>();
    EXPECT_CALL(*gpio, i2c_open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillRepeatedly(Return(tl::expected<std::byte, sucutil::GpioError>(std::byte{ID + 1})));

    auto imu_model = suchm::ImuModel(std::move(gpio));
    auto result = imu_model.begin();
    ASSERT_FALSE(result);
}

TEST(ImuModelBeginTest, fail_on_set_opr_mode_config) {
    // TODO: 順序付けする
    auto gpio = std::make_unique<mock::Gpio>();
    EXPECT_CALL(*gpio, i2c_open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillRepeatedly(Return(tl::expected<std::byte, sucutil::GpioError>(std::byte{ID})));
    EXPECT_CALL(*gpio, i2c_write_byte_data(OPR_MODE_ADDR, OPERATION_MODE_CONFIG))
        .Times(1)
        .WillOnce(Return(tl::make_unexpected(sucutil::GpioError::I2cWriteFailed)));

    auto imu_model = suchm::ImuModel(std::move(gpio));
    auto result = imu_model.begin();
    ASSERT_FALSE(result);
}

TEST(ImuModelBeginTest, fail_on_trigger_reset) {
    // TODO: 順序付けする
    auto gpio = std::make_unique<mock::Gpio>();
    EXPECT_CALL(*gpio, i2c_open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillRepeatedly(Return(tl::expected<std::byte, sucutil::GpioError>(std::byte{ID})));
    EXPECT_CALL(*gpio, i2c_write_byte_data(OPR_MODE_ADDR, OPERATION_MODE_CONFIG))
        .Times(1)
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x20}))
        .Times(1)
        .WillOnce(Return(tl::make_unexpected(sucutil::GpioError::I2cWriteFailed)));

    auto imu_model = suchm::ImuModel(std::move(gpio));
    auto result = imu_model.begin();
    ASSERT_FALSE(result);
}

TEST(ImuModelBeginTest, fail_on_wait_for_reboot) {
    // TODO: 順序付けする
    auto gpio = std::make_unique<mock::Gpio>();
    EXPECT_CALL(*gpio, i2c_open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<std::byte, sucutil::GpioError>(std::byte{ID})))
        .WillRepeatedly(Return(tl::expected<std::byte, sucutil::GpioError>(std::byte{ID + 1})));

    auto imu_model = suchm::ImuModel(std::move(gpio));
    auto result = imu_model.begin();
    ASSERT_FALSE(result);
}

TEST(ImuModelBeginTest, fail_on_set_power_mode_normal) {
    // TODO: 順序付けする
    auto gpio = std::make_unique<mock::Gpio>();
    EXPECT_CALL(*gpio, i2c_open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillRepeatedly(Return(tl::expected<std::byte, sucutil::GpioError>(std::byte{ID})));
    EXPECT_CALL(*gpio, i2c_write_byte_data(OPR_MODE_ADDR, OPERATION_MODE_CONFIG))
        .Times(1)
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x20}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_write_byte_data(PWR_MODE_ADDR, POWER_MODE_NORMAL))
        .Times(1)
        .WillOnce(Return(tl::make_unexpected(sucutil::GpioError::I2cWriteFailed)));

    auto imu_model = suchm::ImuModel(std::move(gpio));
    auto result = imu_model.begin();
    ASSERT_FALSE(result);
}

TEST(ImuModelBeginTest, fail_on_set_page_id) {
    // TODO: 順序付けする
    auto gpio = std::make_unique<mock::Gpio>();
    EXPECT_CALL(*gpio, i2c_open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillRepeatedly(Return(tl::expected<std::byte, sucutil::GpioError>(std::byte{ID})));
    EXPECT_CALL(*gpio, i2c_write_byte_data(OPR_MODE_ADDR, OPERATION_MODE_CONFIG))
        .Times(1)
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x20}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_write_byte_data(PWR_MODE_ADDR, std::byte{POWER_MODE_NORMAL}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_write_byte_data(PAGE_ID_ADDR, std::byte{0x0}))
        .Times(1)
        .WillOnce(Return(tl::make_unexpected(sucutil::GpioError::I2cWriteFailed)));

    auto imu_model = suchm::ImuModel(std::move(gpio));
    auto result = imu_model.begin();
    ASSERT_FALSE(result);
}

TEST(ImuModelBeginTest, fail_on_clear_sys_trigger) {
    // TODO: 順序付けする
    auto gpio = std::make_unique<mock::Gpio>();
    EXPECT_CALL(*gpio, i2c_open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillRepeatedly(Return(tl::expected<std::byte, sucutil::GpioError>(std::byte{ID})));
    EXPECT_CALL(*gpio, i2c_write_byte_data(OPR_MODE_ADDR, OPERATION_MODE_CONFIG))
        .Times(1)
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x20}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_write_byte_data(PWR_MODE_ADDR, std::byte{POWER_MODE_NORMAL}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_write_byte_data(PAGE_ID_ADDR, std::byte{0x0}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x0}))
        .Times(1)
        .WillOnce(Return(tl::make_unexpected(sucutil::GpioError::I2cWriteFailed)));

    auto imu_model = suchm::ImuModel(std::move(gpio));
    auto result = imu_model.begin();
    ASSERT_FALSE(result);
}

TEST(ImuModelBeginTest, fail_on_set_opr_mode_ndof) {
    // TODO: 順序付けする
    auto gpio = std::make_unique<mock::Gpio>();
    EXPECT_CALL(*gpio, i2c_open(ADDRESS))
        .Times(AtLeast(1))
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_read_byte_data(CHIP_ID_ADDR))
        .Times(AtLeast(1))
        .WillRepeatedly(Return(tl::expected<std::byte, sucutil::GpioError>(std::byte{ID})));
    EXPECT_CALL(*gpio, i2c_write_byte_data(OPR_MODE_ADDR, OPERATION_MODE_CONFIG))
        .Times(1)
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x20}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_write_byte_data(PWR_MODE_ADDR, std::byte{POWER_MODE_NORMAL}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_write_byte_data(PAGE_ID_ADDR, std::byte{0x0}))
        .Times(1)
        .WillOnce(Return(tl::expected<void, sucutil::GpioError>()));
    EXPECT_CALL(*gpio, i2c_write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x0}))
        .Times(1)
        .WillOnce(Return(tl::make_unexpected(sucutil::GpioError::I2cWriteFailed)));

    auto imu_model = suchm::ImuModel(std::move(gpio));
    auto result = imu_model.begin();
    ASSERT_FALSE(result);
}

TEST(ImuModelOnReadTest, all) {
    auto gpio = std::make_unique<mock::Gpio>();
    for (int i = 0; i < 6; ++i) {
        // TODO: test with dummy data
        EXPECT_CALL(*gpio, i2c_read_byte_data(EULER_H_LSB_ADDR + i))
            .Times(1)
            .WillOnce(Return(tl::expected<std::byte, sucutil::GpioError>(std::byte{0})));
    }
    // TODO: test with dummy data
    EXPECT_CALL(*gpio, i2c_read_byte_data(TEMP_ADDR))
        .Times(1)
        .WillOnce(Return(tl::expected<std::byte, sucutil::GpioError>(std::byte{0})));

    auto imu_model = suchm::ImuModel(std::move(gpio));
    auto result = imu_model.on_read();
    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
}

TEST(ImuModelOnReadTest, fail_on_read_orientation) {
    auto gpio = std::make_unique<mock::Gpio>();
    EXPECT_CALL(*gpio, i2c_read_byte_data(EULER_H_LSB_ADDR))
        .Times(1)
        .WillOnce(Return(tl::make_unexpected(sucutil::GpioError::I2cReadFailed)));

    auto imu_model = suchm::ImuModel(std::move(gpio));
    auto result = imu_model.on_read();
    ASSERT_FALSE(result);
}

TEST(ImuModelOnReadTest, fail_on_read_temperature) {
    auto gpio = std::make_unique<mock::Gpio>();
    for (int i = 0; i < 6; ++i) {
        EXPECT_CALL(*gpio, i2c_read_byte_data(EULER_H_LSB_ADDR + i))
            .Times(1)
            .WillOnce(Return(tl::expected<std::byte, sucutil::GpioError>(std::byte{0})));
    }
    EXPECT_CALL(*gpio, i2c_read_byte_data(TEMP_ADDR))
        .Times(1)
        .WillOnce(Return(tl::make_unexpected(sucutil::GpioError::I2cReadFailed)));

    auto imu_model = suchm::ImuModel(std::move(gpio));
    auto result = imu_model.on_read();
    ASSERT_FALSE(result);
}

}  // namespace sinsei_umiusi_control::test::hardware_model::imu

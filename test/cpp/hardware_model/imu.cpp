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

namespace sinsei_umiusi_control::test::hardware_model::imu {

using ::testing::InSequence;
using ::testing::Invoke;
using ::testing::Return;

namespace {

using sinsei_umiusi_control::hardware_model::ImuModel;
using sinsei_umiusi_control::hardware_model::interface::I2cDirection;
using sinsei_umiusi_control::hardware_model::interface::I2cMessage;

static constexpr auto BNO055_ADDR = ImuModel::BNO055_ADDR;
static constexpr auto BNO055_ID = ImuModel::BNO055_ID;
static constexpr auto CHIP_ID_ADDR = ImuModel::CHIP_ID_ADDR;
static constexpr auto OPR_MODE_ADDR = ImuModel::OPR_MODE_ADDR;
static constexpr auto SYS_TRIGGER_ADDR = ImuModel::SYS_TRIGGER_ADDR;
static constexpr auto PWR_MODE_ADDR = ImuModel::PWR_MODE_ADDR;
static constexpr auto PAGE_ID_ADDR = ImuModel::PAGE_ID_ADDR;
static constexpr auto OPERATION_MODE_CONFIG = ImuModel::OPERATION_MODE_CONFIG;
static constexpr auto OPERATION_MODE_NDOF = ImuModel::OPERATION_MODE_NDOF;
static constexpr auto POWER_MODE_NORMAL = ImuModel::POWER_MODE_NORMAL;
static constexpr auto ACCEL_ADDR = ImuModel::LINEAR_ACCEL_DATA_X_LSB_ADDR;
static constexpr auto ANGVEL_ADDR = ImuModel::GYRO_DATA_X_LSB_ADDR;
static constexpr auto TEMP_ADDR = ImuModel::TEMP_ADDR;
static constexpr auto QUAT_ADDR = ImuModel::QUATERNION_DATA_W_LSB_ADDR;
static constexpr auto ACCEL_SCALE = 1.0 / 100.0;
static constexpr auto ANGVEL_SCALE = 1.0 / 16.0;
static constexpr auto QUAT_SCALE = 1.0 / (1 << 14);

auto expect_write_reg(
    const I2cMessage * msgs, std::size_t size,
    sinsei_umiusi_control::hardware_model::interface::I2cRegisterAddr reg,
    std::byte value) -> void {
    ASSERT_NE(msgs, nullptr);
    ASSERT_EQ(size, 1U);
    EXPECT_EQ(msgs[0].address.value, BNO055_ADDR.value);
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
    EXPECT_EQ(msgs[0].address.value, BNO055_ADDR.value);
    EXPECT_EQ(msgs[0].direction, I2cDirection::Write);
    ASSERT_EQ(msgs[0].buffer.length, 1U);
    EXPECT_EQ(msgs[0].buffer.data[0], reg.value);
    EXPECT_EQ(msgs[1].address.value, BNO055_ADDR.value);
    EXPECT_EQ(msgs[1].direction, I2cDirection::Read);
    EXPECT_EQ(msgs[1].buffer.length, read_size);
}

auto return_chip_id(const I2cMessage * msgs, std::size_t size, std::byte value)
    -> tl::expected<void, std::string> {
    expect_read_reg(msgs, size, CHIP_ID_ADDR, 1U);
    msgs[1].buffer.data[0] = value;
    return {};
}

auto expect_init_writes_until_reboot(
    const I2cMessage * msgs, std::size_t size, std::size_t call_index)
    -> tl::expected<void, std::string> {
    switch (call_index) {
        case 0:
            return return_chip_id(msgs, size, std::byte{BNO055_ID});
        case 1:
            expect_write_reg(msgs, size, OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
            return {};
        case 2:
            expect_write_reg(msgs, size, SYS_TRIGGER_ADDR, std::byte{0x20});
            return {};
        case 3:
            return return_chip_id(msgs, size, std::byte{BNO055_ID});
        default:
            return tl::make_unexpected("unexpected init call");
    }
}

}  // namespace

TEST(ImuModelTest, OnInitSuccess) {
    auto mock_i2c = std::make_unique<mock::I2c>();

    EXPECT_CALL(*mock_i2c, open()).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*mock_i2c, transfer(testing::_, testing::_))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            return return_chip_id(msgs, size, std::byte{BNO055_ID});
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            expect_write_reg(msgs, size, OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
            return tl::expected<void, std::string>{};
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            expect_write_reg(msgs, size, SYS_TRIGGER_ADDR, std::byte{0x20});
            return tl::expected<void, std::string>{};
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            return return_chip_id(msgs, size, std::byte{BNO055_ID});
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            expect_write_reg(msgs, size, PWR_MODE_ADDR, POWER_MODE_NORMAL);
            return tl::expected<void, std::string>{};
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            expect_write_reg(msgs, size, PAGE_ID_ADDR, std::byte{0x00});
            return tl::expected<void, std::string>{};
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            expect_write_reg(msgs, size, SYS_TRIGGER_ADDR, std::byte{0x00});
            return tl::expected<void, std::string>{};
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            expect_write_reg(msgs, size, OPR_MODE_ADDR, OPERATION_MODE_NDOF);
            return tl::expected<void, std::string>{};
        }));

    sinsei_umiusi_control::hardware_model::ImuModel imu_model(std::move(mock_i2c));
    const auto res = imu_model.on_init();

    ASSERT_TRUE(res);
}

TEST(ImuModelTest, OnInitFailOnI2cOpen) {
    auto mock_i2c = std::make_unique<mock::I2c>();

    EXPECT_CALL(*mock_i2c, open()).WillOnce(Return(tl::make_unexpected("open failed")));

    sinsei_umiusi_control::hardware_model::ImuModel imu_model(std::move(mock_i2c));
    const auto res = imu_model.on_init();

    ASSERT_FALSE(res);
}

TEST(ImuModelTest, OnInitFailOnWrongChipId) {
    auto mock_i2c = std::make_unique<mock::I2c>();

    EXPECT_CALL(*mock_i2c, open()).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*mock_i2c, transfer(testing::_, testing::_))
        .Times(2)
        .WillRepeatedly(Invoke([](const I2cMessage * msgs, std::size_t size) {
            return return_chip_id(msgs, size, std::byte{static_cast<uint8_t>(BNO055_ID + 1)});
        }));

    auto imu_model = ImuModel(std::move(mock_i2c));
    auto res = imu_model.on_init();

    ASSERT_FALSE(res);
}

TEST(ImuModelTest, OnInitFailOnWaitForReboot) {
    auto _ = InSequence{};

    auto mock_i2c = std::make_unique<mock::I2c>();

    EXPECT_CALL(*mock_i2c, open()).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*mock_i2c, transfer(testing::_, testing::_))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            return return_chip_id(msgs, size, std::byte{BNO055_ID});
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            expect_write_reg(msgs, size, OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
            return tl::expected<void, std::string>{};
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            expect_write_reg(msgs, size, SYS_TRIGGER_ADDR, std::byte{0x20});
            return tl::expected<void, std::string>{};
        }));
    EXPECT_CALL(*mock_i2c, transfer(testing::_, testing::_))
        .Times(100)
        .WillRepeatedly(Invoke([](const I2cMessage * msgs, std::size_t size) {
            return return_chip_id(msgs, size, std::byte{0x00});
        }));

    auto imu_model = ImuModel(std::move(mock_i2c));
    auto res = imu_model.on_init();

    ASSERT_FALSE(res);
}

TEST(ImuModelTest, OnInitFailOnSetNormalPowerMode) {
    auto mock_i2c = std::make_unique<mock::I2c>();

    EXPECT_CALL(*mock_i2c, open()).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*mock_i2c, transfer(testing::_, testing::_))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            return expect_init_writes_until_reboot(msgs, size, 0);
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            return expect_init_writes_until_reboot(msgs, size, 1);
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            return expect_init_writes_until_reboot(msgs, size, 2);
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            return expect_init_writes_until_reboot(msgs, size, 3);
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            expect_write_reg(msgs, size, PWR_MODE_ADDR, POWER_MODE_NORMAL);
            return tl::make_unexpected("power mode failed");
        }));

    auto imu_model = ImuModel(std::move(mock_i2c));
    auto res = imu_model.on_init();

    ASSERT_FALSE(res);
}

TEST(ImuModelTest, OnInitFailOnSetPageId) {
    auto mock_i2c = std::make_unique<mock::I2c>();

    EXPECT_CALL(*mock_i2c, open()).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*mock_i2c, transfer(testing::_, testing::_))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            return expect_init_writes_until_reboot(msgs, size, 0);
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            return expect_init_writes_until_reboot(msgs, size, 1);
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            return expect_init_writes_until_reboot(msgs, size, 2);
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            return expect_init_writes_until_reboot(msgs, size, 3);
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            expect_write_reg(msgs, size, PWR_MODE_ADDR, POWER_MODE_NORMAL);
            return tl::expected<void, std::string>{};
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            expect_write_reg(msgs, size, PAGE_ID_ADDR, std::byte{0x00});
            return tl::make_unexpected("page id failed");
        }));

    auto imu_model = ImuModel(std::move(mock_i2c));
    auto res = imu_model.on_init();

    ASSERT_FALSE(res);
}

TEST(ImuModelTest, OnInitFailOnClearSysTrigger) {
    auto mock_i2c = std::make_unique<mock::I2c>();

    EXPECT_CALL(*mock_i2c, open()).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*mock_i2c, transfer(testing::_, testing::_))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            return expect_init_writes_until_reboot(msgs, size, 0);
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            return expect_init_writes_until_reboot(msgs, size, 1);
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            return expect_init_writes_until_reboot(msgs, size, 2);
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            return expect_init_writes_until_reboot(msgs, size, 3);
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            expect_write_reg(msgs, size, PWR_MODE_ADDR, POWER_MODE_NORMAL);
            return tl::expected<void, std::string>{};
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            expect_write_reg(msgs, size, PAGE_ID_ADDR, std::byte{0x00});
            return tl::expected<void, std::string>{};
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            expect_write_reg(msgs, size, SYS_TRIGGER_ADDR, std::byte{0x00});
            return tl::make_unexpected("clear sys trigger failed");
        }));

    auto imu_model = ImuModel(std::move(mock_i2c));
    auto res = imu_model.on_init();

    ASSERT_FALSE(res);
}

TEST(ImuModelTest, OnReadSuccess) {
    auto mock_i2c = std::make_unique<mock::I2c>();

    static constexpr size_t FRAME_LENGTH =
        std::to_integer<size_t>(TEMP_ADDR.value) - std::to_integer<size_t>(ANGVEL_ADDR.value) + 1;
    auto mock_data = std::array<std::byte, FRAME_LENGTH>{};

    const auto write_s16 = [&mock_data](size_t offset, int16_t value) {
        const auto raw = static_cast<uint16_t>(value);
        mock_data[offset] = std::byte{static_cast<uint8_t>(raw & 0xFF)};
        mock_data[offset + 1] = std::byte{static_cast<uint8_t>((raw >> 8) & 0xFF)};
    };

    constexpr auto OFFSET_QUAT =
        std::to_integer<size_t>(QUAT_ADDR.value) - std::to_integer<size_t>(ANGVEL_ADDR.value);
    constexpr auto OFFSET_LINEAR_ACCEL =
        std::to_integer<size_t>(ACCEL_ADDR.value) - std::to_integer<size_t>(ANGVEL_ADDR.value);
    constexpr auto OFFSET_TEMP =
        std::to_integer<size_t>(TEMP_ADDR.value) - std::to_integer<size_t>(ANGVEL_ADDR.value);

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

    EXPECT_CALL(*mock_i2c, transfer(testing::_, testing::_))
        .WillOnce(Invoke(
            [&mock_data](
                const I2cMessage * msgs, std::size_t size) -> tl::expected<void, std::string> {
                EXPECT_EQ(msgs[0].buffer.data[0], ANGVEL_ADDR.value);
                EXPECT_EQ(msgs[1].buffer.length, mock_data.size());
                if (size != 2U) {
                    return tl::make_unexpected("unexpected transfer size");
                }
                std::memcpy(msgs[1].buffer.data, mock_data.data(), mock_data.size());
                return {};
            }));

    sinsei_umiusi_control::hardware_model::ImuModel imu_model(std::move(mock_i2c));
    const auto res = imu_model.on_read();

    ASSERT_TRUE(res);
    const auto [quaternion, acceleration, angular_velocity, temperature] = res.value();

    EXPECT_DOUBLE_EQ(angular_velocity.x, static_cast<double>(GYRO_X_RAW) * ANGVEL_SCALE);
    EXPECT_DOUBLE_EQ(angular_velocity.y, static_cast<double>(GYRO_Y_RAW) * ANGVEL_SCALE);
    EXPECT_DOUBLE_EQ(angular_velocity.z, static_cast<double>(GYRO_Z_RAW) * ANGVEL_SCALE);

    EXPECT_DOUBLE_EQ(quaternion.x, static_cast<double>(QUAT_X_RAW) * QUAT_SCALE);
    EXPECT_DOUBLE_EQ(quaternion.y, static_cast<double>(QUAT_Y_RAW) * QUAT_SCALE);
    EXPECT_DOUBLE_EQ(quaternion.z, static_cast<double>(QUAT_Z_RAW) * QUAT_SCALE);
    EXPECT_DOUBLE_EQ(quaternion.w, static_cast<double>(QUAT_W_RAW) * QUAT_SCALE);

    EXPECT_DOUBLE_EQ(acceleration.x, static_cast<double>(ACCEL_X_RAW) * ACCEL_SCALE);
    EXPECT_DOUBLE_EQ(acceleration.y, static_cast<double>(ACCEL_Y_RAW) * ACCEL_SCALE);
    EXPECT_DOUBLE_EQ(acceleration.z, static_cast<double>(ACCEL_Z_RAW) * ACCEL_SCALE);

    EXPECT_EQ(temperature.value, static_cast<int8_t>(TEMP_RAW));
}

TEST(ImuModelTest, OnReadFailOnTransfer) {
    auto mock_i2c = std::make_unique<mock::I2c>();

    EXPECT_CALL(*mock_i2c, transfer(testing::_, testing::_))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            expect_read_reg(
                msgs, size, sinsei_umiusi_control::hardware_model::interface::I2cRegisterAddr{
                                ANGVEL_ADDR.value},
                std::to_integer<size_t>(TEMP_ADDR.value) - std::to_integer<size_t>(ANGVEL_ADDR.value) +
                    1);
            return tl::make_unexpected("read failed");
        }));

    sinsei_umiusi_control::hardware_model::ImuModel imu_model(std::move(mock_i2c));
    const auto res = imu_model.on_read();

    ASSERT_FALSE(res);
}

TEST(ImuModelTest, OnDestroySuccess) {
    auto mock_i2c = std::make_unique<mock::I2c>();

    EXPECT_CALL(*mock_i2c, close()).WillOnce(Return(tl::expected<void, std::string>{}));

    sinsei_umiusi_control::hardware_model::ImuModel imu_model(std::move(mock_i2c));
    const auto res = imu_model.on_destroy();

    ASSERT_TRUE(res);
}

TEST(ImuModelTest, OnDestroyFail) {
    auto mock_i2c = std::make_unique<mock::I2c>();

    EXPECT_CALL(*mock_i2c, close()).WillOnce(Return(tl::make_unexpected("not open")));

    sinsei_umiusi_control::hardware_model::ImuModel imu_model(std::move(mock_i2c));
    const auto res = imu_model.on_destroy();

    ASSERT_FALSE(res);
}

}  // namespace sinsei_umiusi_control::test::hardware_model::imu

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>

#include "mock/i2c.hpp"
#include "sinsei_umiusi_control/hardware_model/imu/bno055_model.hpp"
#include "sinsei_umiusi_control/state/imu.hpp"

namespace sinsei_umiusi_control::test::hardware_model::bno055 {

using ::testing::DoAll;
using ::testing::InSequence;
using ::testing::Invoke;
using ::testing::Return;

namespace {

using sinsei_umiusi_control::hardware_model::imu::Bno055Model;
using sinsei_umiusi_control::hardware_model::interface::I2cDirection;
using sinsei_umiusi_control::hardware_model::interface::I2cMessage;

constexpr auto ACCEL_SCALE = 1.0 / 100.0;
constexpr auto ANGVEL_SCALE = 1.0 / 16.0;
constexpr auto QUAT_SCALE = 1.0 / (1 << 14);
constexpr auto _ = ::testing::_;

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
    sinsei_umiusi_control::hardware_model::interface::I2cRegisterAddr reg, std::size_t read_size)
    -> void {
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

auto expect_init_writes_until_reboot(
    const I2cMessage * msgs, std::size_t size, std::size_t call_index)
    -> tl::expected<void, std::string> {
    switch (call_index) {
        case 0:
            return return_chip_id(msgs, size, std::byte{Bno055Model::ID});
        case 1:
            expect_write_reg(
                msgs, size, Bno055Model::OPR_MODE_ADDR, Bno055Model::OPERATION_MODE_CONFIG);
            return {};
        case 2:
            expect_write_reg(msgs, size, Bno055Model::SYS_TRIGGER_ADDR, std::byte{0x20});
            return {};
        case 3:
            return return_chip_id(msgs, size, std::byte{Bno055Model::ID});
        default:
            return tl::make_unexpected("unexpected init call");
    }
}

}  // namespace

TEST(Bno055ModelBeginTest, success) {
    auto sequence = InSequence{};

    auto i2c = std::make_unique<mock::I2c>();

    EXPECT_CALL(*i2c, open()).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*i2c, transfer(_, _))
        .WillOnce(Invoke(
            [](const I2cMessage * msgs, std::size_t size) {
                return return_chip_id(msgs, size, std::byte{Bno055Model::ID});
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
        .WillOnce(Invoke(
            [](const I2cMessage * msgs, std::size_t size) {
                return return_chip_id(msgs, size, std::byte{Bno055Model::ID});
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

    auto bno055_model = Bno055Model(std::move(i2c));
    auto result = bno055_model.begin();

    ASSERT_TRUE(result) << std::string("Error: ") + result.error();
}

TEST(Bno055ModelBeginTest, fail_on_open) {
    auto i2c = std::make_unique<mock::I2c>();

    EXPECT_CALL(*i2c, open()).WillOnce(Return(tl::make_unexpected("open failed")));

    auto bno055_model = Bno055Model(std::move(i2c));
    auto result = bno055_model.begin();

    ASSERT_FALSE(result);
}

TEST(Bno055ModelBeginTest, fail_on_read_chip_id) {
    auto i2c = std::make_unique<mock::I2c>();

    EXPECT_CALL(*i2c, open()).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*i2c, transfer(_, _))
        .Times(2)
        .WillRepeatedly(Invoke([](const I2cMessage * msgs, std::size_t size) {
            expect_read_reg(msgs, size, Bno055Model::CHIP_ID_ADDR, 1U);
            return tl::make_unexpected("read failed");
        }));

    auto bno055_model = Bno055Model(std::move(i2c));
    auto result = bno055_model.begin();

    ASSERT_FALSE(result);
}

TEST(Bno055ModelBeginTest, fail_on_wrong_chip_id) {
    auto i2c = std::make_unique<mock::I2c>();

    EXPECT_CALL(*i2c, open()).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*i2c, transfer(_, _))
        .Times(2)
        .WillRepeatedly(Invoke([](const I2cMessage * msgs, std::size_t size) {
            return return_chip_id(
                msgs, size, std::byte{static_cast<uint8_t>(Bno055Model::ID + 1)});
        }));

    auto bno055_model = Bno055Model(std::move(i2c));
    auto result = bno055_model.begin();

    ASSERT_FALSE(result);
}

TEST(Bno055ModelBeginTest, fail_on_set_opr_mode_config) {
    auto i2c = std::make_unique<mock::I2c>();

    EXPECT_CALL(*i2c, open()).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*i2c, transfer(_, _))
        .WillOnce(Invoke(
            [](const I2cMessage * msgs, std::size_t size) {
                return return_chip_id(msgs, size, std::byte{Bno055Model::ID});
            }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            expect_write_reg(
                msgs, size, Bno055Model::OPR_MODE_ADDR, Bno055Model::OPERATION_MODE_CONFIG);
            return tl::make_unexpected("config failed");
        }));

    auto bno055_model = Bno055Model(std::move(i2c));
    auto result = bno055_model.begin();

    ASSERT_FALSE(result);
}

TEST(Bno055ModelBeginTest, fail_on_trigger_reset) {
    auto i2c = std::make_unique<mock::I2c>();

    EXPECT_CALL(*i2c, open()).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*i2c, transfer(_, _))
        .WillOnce(Invoke(
            [](const I2cMessage * msgs, std::size_t size) {
                return return_chip_id(msgs, size, std::byte{Bno055Model::ID});
            }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            expect_write_reg(
                msgs, size, Bno055Model::OPR_MODE_ADDR, Bno055Model::OPERATION_MODE_CONFIG);
            return tl::expected<void, std::string>{};
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            expect_write_reg(msgs, size, Bno055Model::SYS_TRIGGER_ADDR, std::byte{0x20});
            return tl::make_unexpected("reset failed");
        }));

    auto bno055_model = Bno055Model(std::move(i2c));
    auto result = bno055_model.begin();

    ASSERT_FALSE(result);
}

TEST(Bno055ModelBeginTest, fail_on_wait_for_reboot) {
    auto i2c = std::make_unique<mock::I2c>();

    EXPECT_CALL(*i2c, open()).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*i2c, transfer(_, _))
        .Times(103)
        .WillOnce(Invoke(
            [](const I2cMessage * msgs, std::size_t size) {
                return return_chip_id(msgs, size, std::byte{Bno055Model::ID});
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
        .WillRepeatedly(Invoke([](const I2cMessage * msgs, std::size_t size) {
            return return_chip_id(msgs, size, std::byte{0x00});
        }));

    auto bno055_model = Bno055Model(std::move(i2c));
    auto result = bno055_model.begin();

    ASSERT_FALSE(result);
}

TEST(Bno055ModelBeginTest, fail_on_set_power_mode_normal) {
    auto i2c = std::make_unique<mock::I2c>();

    EXPECT_CALL(*i2c, open()).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*i2c, transfer(_, _))
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
            expect_write_reg(
                msgs, size, Bno055Model::PWR_MODE_ADDR, Bno055Model::POWER_MODE_NORMAL);
            return tl::make_unexpected("power mode failed");
        }));

    auto bno055_model = Bno055Model(std::move(i2c));
    auto result = bno055_model.begin();

    ASSERT_FALSE(result);
}

TEST(Bno055ModelBeginTest, fail_on_set_page_id) {
    auto i2c = std::make_unique<mock::I2c>();

    EXPECT_CALL(*i2c, open()).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*i2c, transfer(_, _))
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
            expect_write_reg(
                msgs, size, Bno055Model::PWR_MODE_ADDR, Bno055Model::POWER_MODE_NORMAL);
            return tl::expected<void, std::string>{};
        }))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            expect_write_reg(msgs, size, Bno055Model::PAGE_ID_ADDR, std::byte{0x00});
            return tl::make_unexpected("page id failed");
        }));

    auto bno055_model = Bno055Model(std::move(i2c));
    auto result = bno055_model.begin();

    ASSERT_FALSE(result);
}

TEST(Bno055ModelBeginTest, fail_on_clear_sys_trigger) {
    auto i2c = std::make_unique<mock::I2c>();

    EXPECT_CALL(*i2c, open()).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*i2c, transfer(_, _))
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
            return tl::make_unexpected("clear sys trigger failed");
        }));

    auto bno055_model = Bno055Model(std::move(i2c));
    auto result = bno055_model.begin();

    ASSERT_FALSE(result);
}

TEST(Bno055ModelBeginTest, fail_on_set_opr_mode_ndof) {
    auto i2c = std::make_unique<mock::I2c>();

    EXPECT_CALL(*i2c, open()).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*i2c, transfer(_, _))
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
            return tl::make_unexpected("ndof failed");
        }));

    auto bno055_model = Bno055Model(std::move(i2c));
    auto result = bno055_model.begin();

    ASSERT_FALSE(result);
}

TEST(Bno055ModelGetAccelerationTest, success) {
    auto i2c = std::make_unique<mock::I2c>();

    constexpr std::byte MOCK_DATA[6] = {std::byte{0x00}, std::byte{0x10}, std::byte{0x00},
                                        std::byte{0x20}, std::byte{0x00}, std::byte{0x30}};

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

    EXPECT_CALL(*i2c, transfer(_, _))
        .WillOnce(Invoke([&](const I2cMessage * msgs, std::size_t size) {
            expect_read_reg(msgs, size, Bno055Model::LINEAR_ACCEL_DATA_X_LSB_ADDR, 6U);
            std::memcpy(msgs[1].buffer.data, MOCK_DATA, sizeof(MOCK_DATA));
            return tl::expected<void, std::string>{};
        }));

    auto bno055_model = Bno055Model{std::move(i2c)};
    const auto result = bno055_model.get_acceleration();

    ASSERT_TRUE(result);
    EXPECT_EQ(result.value().x, expected_accel.x);
    EXPECT_EQ(result.value().y, expected_accel.y);
    EXPECT_EQ(result.value().z, expected_accel.z);
}

TEST(Bno055ModelGetAccelerationTest, fail_on_get_vector) {
    auto i2c = std::make_unique<mock::I2c>();

    EXPECT_CALL(*i2c, transfer(_, _))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            expect_read_reg(msgs, size, Bno055Model::LINEAR_ACCEL_DATA_X_LSB_ADDR, 6U);
            return tl::make_unexpected("read failed");
        }));

    auto bno055_model = Bno055Model{std::move(i2c)};
    const auto result = bno055_model.get_acceleration();

    ASSERT_FALSE(result);
}

TEST(Bno055ModelGetAngularVelocity, success) {
    auto i2c = std::make_unique<mock::I2c>();

    constexpr std::byte MOCK_DATA[6] = {std::byte{0x00}, std::byte{0x10}, std::byte{0x00},
                                        std::byte{0x20}, std::byte{0x00}, std::byte{0x30}};

    EXPECT_CALL(*i2c, transfer(_, _))
        .WillOnce(Invoke([&](const I2cMessage * msgs, std::size_t size) {
            expect_read_reg(msgs, size, Bno055Model::GYRO_DATA_X_LSB_ADDR, 6U);
            std::memcpy(msgs[1].buffer.data, MOCK_DATA, sizeof(MOCK_DATA));
            return tl::expected<void, std::string>{};
        }));

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

    auto bno055_model = Bno055Model{std::move(i2c)};
    const auto result = bno055_model.get_angular_velocity();

    ASSERT_TRUE(result);
    EXPECT_EQ(result.value().x, expected_angular_vel.x);
    EXPECT_EQ(result.value().y, expected_angular_vel.y);
    EXPECT_EQ(result.value().z, expected_angular_vel.z);
}

TEST(Bno055ModelGetTempTest, success) {
    auto i2c = std::make_unique<mock::I2c>();

    constexpr auto MOCK_DATA = std::byte{0x12};
    const auto expected_temp =
        state::imu::Temperature{std::to_integer<int8_t>(MOCK_DATA & std::byte{0x7F})};

    EXPECT_CALL(*i2c, transfer(_, _))
        .WillOnce(Invoke([&](const I2cMessage * msgs, std::size_t size) {
            expect_read_reg(msgs, size, Bno055Model::TEMP_ADDR, 1U);
            msgs[1].buffer.data[0] = MOCK_DATA;
            return tl::expected<void, std::string>{};
        }));

    auto bno055_model = Bno055Model{std::move(i2c)};
    const auto result = bno055_model.get_temp();

    ASSERT_TRUE(result);
    ASSERT_EQ(result.value().value, expected_temp.value);
}

TEST(Bno055ModelGetTempTest, fail_on_get_temperature) {
    auto i2c = std::make_unique<mock::I2c>();

    EXPECT_CALL(*i2c, transfer(_, _))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            expect_read_reg(msgs, size, Bno055Model::TEMP_ADDR, 1U);
            return tl::make_unexpected("read failed");
        }));

    auto bno055_model = Bno055Model(std::move(i2c));
    auto result = bno055_model.get_temp();

    ASSERT_FALSE(result);
}

TEST(Bno055ModelGetQuatTest, success) {
    auto i2c = std::make_unique<mock::I2c>();

    constexpr std::byte MOCK_DATA[8] = {std::byte{0x00}, std::byte{0x10}, std::byte{0x00},
                                        std::byte{0x20}, std::byte{0x00}, std::byte{0x30},
                                        std::byte{0x00}, std::byte{0x40}};

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

    EXPECT_CALL(*i2c, transfer(_, _))
        .WillOnce(Invoke([&](const I2cMessage * msgs, std::size_t size) {
            expect_read_reg(msgs, size, Bno055Model::QUATERNION_DATA_W_LSB_ADDR, 8U);
            std::memcpy(msgs[1].buffer.data, MOCK_DATA, sizeof(MOCK_DATA));
            return tl::expected<void, std::string>{};
        }));

    auto bno055_model = Bno055Model{std::move(i2c)};
    const auto result = bno055_model.get_quat();

    ASSERT_TRUE(result);
    EXPECT_EQ(result.value().x, expected_quat.x);
    EXPECT_EQ(result.value().y, expected_quat.y);
    EXPECT_EQ(result.value().z, expected_quat.z);
    EXPECT_EQ(result.value().w, expected_quat.w);
}

TEST(Bno055ModelGetQuatTest, fail_on_get_quaternion) {
    auto i2c = std::make_unique<mock::I2c>();

    EXPECT_CALL(*i2c, transfer(_, _))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            expect_read_reg(msgs, size, Bno055Model::QUATERNION_DATA_W_LSB_ADDR, 8U);
            return tl::make_unexpected("read failed");
        }));

    auto bno055_model = Bno055Model(std::move(i2c));
    auto result = bno055_model.get_quat();

    ASSERT_FALSE(result);
}

}  // namespace sinsei_umiusi_control::test::hardware_model::bno055

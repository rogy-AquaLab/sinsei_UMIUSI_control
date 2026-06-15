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

auto expect_init_writes_until_reboot(
    const I2cMessage * msgs, std::size_t size,
    std::size_t call_index) -> tl::expected<void, std::string> {
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
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
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
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
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
            return return_chip_id(msgs, size, std::byte{static_cast<uint8_t>(Bno055Model::ID + 1)});
        }));

    auto bno055_model = Bno055Model(std::move(i2c));
    auto result = bno055_model.begin();

    ASSERT_FALSE(result);
}

TEST(Bno055ModelBeginTest, fail_on_set_opr_mode_config) {
    auto i2c = std::make_unique<mock::I2c>();

    EXPECT_CALL(*i2c, open()).WillOnce(Return(tl::expected<void, std::string>{}));
    EXPECT_CALL(*i2c, transfer(_, _))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
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
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
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
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
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

TEST(Bno055ModelReadTest, success) {
    auto i2c = std::make_unique<mock::I2c>();

    auto read_buffer = std::array<std::byte, Bno055Model::FRAME_LENGTH>{};
    constexpr std::byte MOCK_DATA_GYRO[6] = {std::byte{0x00}, std::byte{0x10}, std::byte{0x00},
                                             std::byte{0x20}, std::byte{0x00}, std::byte{0x30}};
    constexpr std::byte MOCK_DATA_QUAT[8] = {std::byte{0x00}, std::byte{0x40}, std::byte{0x00},
                                             std::byte{0x30}, std::byte{0x00}, std::byte{0x20},
                                             std::byte{0x00}, std::byte{0x10}};
    constexpr std::byte MOCK_DATA_ACCEL[6] = {std::byte{0x00}, std::byte{0x64}, std::byte{0x00},
                                              std::byte{0xC8}, std::byte{0x01}, std::byte{0x2C}};
    constexpr auto MOCK_TEMP = std::byte{0x85};

    std::memcpy(
        read_buffer.data() + Bno055Model::OFFSET_GYRO, MOCK_DATA_GYRO, sizeof(MOCK_DATA_GYRO));
    std::memcpy(
        read_buffer.data() + Bno055Model::OFFSET_QUAT, MOCK_DATA_QUAT, sizeof(MOCK_DATA_QUAT));
    std::memcpy(
        read_buffer.data() + Bno055Model::OFFSET_LINEAR_ACCEL, MOCK_DATA_ACCEL,
        sizeof(MOCK_DATA_ACCEL));
    read_buffer[Bno055Model::OFFSET_TEMP] = MOCK_TEMP;

    EXPECT_CALL(*i2c, transfer(_, _))
        .WillOnce(Invoke([&](const I2cMessage * msgs, std::size_t size) {
            expect_read_frame(msgs, size);
            std::memcpy(msgs[1].buffer.data, read_buffer.data(), read_buffer.size());
            return tl::expected<void, std::string>{};
        }));

    auto bno055_model = Bno055Model{std::move(i2c)};
    const auto result = bno055_model.read();

    ASSERT_TRUE(result);
    const auto & [quaternion, acceleration, angular_velocity, temperature] = result.value();
    EXPECT_EQ(quaternion.x, 0.75);
    EXPECT_EQ(quaternion.y, 0.5);
    EXPECT_EQ(quaternion.z, 0.25);
    EXPECT_EQ(quaternion.w, 1.0);
    EXPECT_EQ(acceleration.x, 256.0);
    EXPECT_EQ(acceleration.y, -143.36);
    EXPECT_EQ(acceleration.z, 112.65);
    EXPECT_EQ(angular_velocity.x, 256.0);
    EXPECT_EQ(angular_velocity.y, 512.0);
    EXPECT_EQ(angular_velocity.z, 768.0);
    EXPECT_EQ(temperature.value, -123);
}

TEST(Bno055ModelReadTest, fail_on_read_frame) {
    auto i2c = std::make_unique<mock::I2c>();

    EXPECT_CALL(*i2c, transfer(_, _))
        .WillOnce(Invoke([](const I2cMessage * msgs, std::size_t size) {
            expect_read_frame(msgs, size);
            return tl::make_unexpected("read failed");
        }));

    auto bno055_model = Bno055Model{std::move(i2c)};
    const auto result = bno055_model.read();

    ASSERT_FALSE(result);
}

}  // namespace sinsei_umiusi_control::test::hardware_model::bno055

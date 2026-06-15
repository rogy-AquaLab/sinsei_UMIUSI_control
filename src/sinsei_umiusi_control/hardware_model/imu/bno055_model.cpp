#include "sinsei_umiusi_control/hardware_model/imu/bno055_model.hpp"

#include <array>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

using namespace sinsei_umiusi_control::hardware_model;

// ref: https://github.com/adafruit/Adafruit_BNO055/blob/1b1af09/Adafruit_BNO055.cpp

imu::Bno055Model::Bno055Model(std::unique_ptr<interface::I2c> i2c) : i2c(std::move(i2c)) {}

auto imu::Bno055Model::write_reg(RegisterAddr reg, std::byte value)
    -> tl::expected<void, std::string> {
    std::byte data[] = {reg.value, value};
    interface::I2cMessage msg{ADDRESS, interface::I2cDirection::Write, {data, 2}, 0};
    return this->i2c->transfer(&msg, 1);
}

auto imu::Bno055Model::read_reg(RegisterAddr reg, interface::I2cBufferView buffer)
    -> tl::expected<void, std::string> {
    std::byte reg_addr[] = {reg.value};
    interface::I2cMessage msgs[2] = {
        {ADDRESS, interface::I2cDirection::Write, {reg_addr, 1}, 0},
        {ADDRESS, interface::I2cDirection::Read, buffer, 0}};
    return this->i2c->transfer(msgs, 2);
}

auto imu::Bno055Model::begin() -> tl::expected<void, std::string> {
    auto res = this->i2c->open();
    if (!res) {
        return tl::make_unexpected("Failed to open I2C bus for BNO055: " + res.error());
    }

    auto read_id = [&]() -> tl::expected<uint8_t, std::string> {
        std::byte value{};
        return this->read_reg(CHIP_ID_ADDR, {&value, 1}).map([&value]() {
            return std::to_integer<uint8_t>(value);
        });
    };

    auto id_res = read_id();
    if (!id_res || id_res.value() != ID) {
        rclcpp::sleep_for(1000ms);
        id_res = read_id();
        if (!id_res) {
            return tl::make_unexpected("Failed to read CHIP_ID from BNO055: " + id_res.error());
        }
        if (id_res.value() != ID) {
            return tl::make_unexpected(
                "Failed to find BNO055 device (found ID: " + std::to_string(id_res.value()) + ")");
        }
    }

    res = this->write_reg(OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
    if (!res) {
        return tl::make_unexpected("Failed to set BNO055 to CONFIG mode: " + res.error());
    }

    res = this->write_reg(SYS_TRIGGER_ADDR, std::byte{0x20});
    if (!res) {
        return tl::make_unexpected("Failed to trigger BNO055 reset: " + res.error());
    }

    rclcpp::sleep_for(30ms);
    constexpr int TIMEOUT_MS = 1000;
    constexpr int WAIT_INTERVAL_MS = 10;
    bool timeout = true;
    for (int time = 0; time < TIMEOUT_MS; time += WAIT_INTERVAL_MS) {
        auto current_id = read_id();
        if (current_id && current_id.value() == ID) {
            timeout = false;
            break;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(WAIT_INTERVAL_MS));
    }
    if (timeout) {
        return tl::make_unexpected(
            "BNO055 did not restart within timeout period (" + std::to_string(TIMEOUT_MS) + " ms)");
    }
    rclcpp::sleep_for(50ms);

    res = this->write_reg(PWR_MODE_ADDR, POWER_MODE_NORMAL);
    if (!res) {
        return tl::make_unexpected("Failed to set BNO055 to NORMAL power mode: " + res.error());
    }
    rclcpp::sleep_for(10ms);

    res = this->write_reg(PAGE_ID_ADDR, std::byte{0x00});
    if (!res) {
        return tl::make_unexpected("Failed to set BNO055 to PAGE 0: " + res.error());
    }

    res = this->write_reg(SYS_TRIGGER_ADDR, std::byte{0x00});
    if (!res) {
        return tl::make_unexpected("Failed to clear BNO055 SYS_TRIGGER: " + res.error());
    }
    rclcpp::sleep_for(10ms);

    res = this->write_reg(OPR_MODE_ADDR, OPERATION_MODE_NDOF);
    if (!res) {
        return tl::make_unexpected("Failed to set BNO055 to NDOF mode: " + res.error());
    }
    rclcpp::sleep_for(20ms);

    return {};
}

auto imu::Bno055Model::close() -> tl::expected<void, std::string> {
    auto res = this->i2c->close();
    if (!res) {
        return tl::make_unexpected("Failed to close I2C bus: " + res.error());
    }
    return {};
}

auto imu::Bno055Model::read() -> tl::expected<ReadResult, std::string> {
    auto buffer = std::array<std::byte, FRAME_LENGTH>{};
    const auto res = this->read_reg(FRAME_START, {buffer.data(), buffer.size()});
    if (!res) {
        return tl::make_unexpected("I2C read error: " + res.error());
    }

    constexpr auto GYRO_SCALE = 1.0 / 16.0;
    const auto angular_velocity = state::imu::AngularVelocity{
        static_cast<double>(to_s16(&buffer[OFFSET_GYRO + 0])) * GYRO_SCALE,
        static_cast<double>(to_s16(&buffer[OFFSET_GYRO + 2])) * GYRO_SCALE,
        static_cast<double>(to_s16(&buffer[OFFSET_GYRO + 4])) * GYRO_SCALE};

    constexpr auto QUAT_SCALE = 1.0 / (1 << 14);
    const auto quaternion = state::imu::Quaternion{
        static_cast<double>(to_s16(&buffer[OFFSET_QUAT + 2])) * QUAT_SCALE,
        static_cast<double>(to_s16(&buffer[OFFSET_QUAT + 4])) * QUAT_SCALE,
        static_cast<double>(to_s16(&buffer[OFFSET_QUAT + 6])) * QUAT_SCALE,
        static_cast<double>(to_s16(&buffer[OFFSET_QUAT + 0])) * QUAT_SCALE};

    constexpr auto LINEAR_ACCEL_SCALE = 1.0 / 100.0;
    const auto acceleration = state::imu::Acceleration{
        static_cast<double>(to_s16(&buffer[OFFSET_LINEAR_ACCEL + 0])) * LINEAR_ACCEL_SCALE,
        static_cast<double>(to_s16(&buffer[OFFSET_LINEAR_ACCEL + 2])) * LINEAR_ACCEL_SCALE,
        static_cast<double>(to_s16(&buffer[OFFSET_LINEAR_ACCEL + 4])) * LINEAR_ACCEL_SCALE};

    const auto temperature = state::imu::Temperature{std::to_integer<int8_t>(buffer[OFFSET_TEMP])};

    return ReadResult{quaternion, acceleration, angular_velocity, temperature};
}

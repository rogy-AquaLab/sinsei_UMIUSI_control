#include "sinsei_umiusi_control/hardware_model/imu/bno055_model.hpp"

#include <array>
#include <boost/math/constants/constants.hpp>
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

auto imu::Bno055Model::validate_id() -> tl::expected<void, std::string> {
    std::byte value{};
    const auto res = this->read_reg(CHIP_ID_ADDR, {&value, 1});
    if (!res) {
        return tl::make_unexpected("Failed to read CHIP_ID from BNO055: " + res.error());
    }

    const auto id = std::to_integer<uint8_t>(value);
    if (id != CHIP_ID) {
        return tl::make_unexpected(
            "Failed to find BNO055 device (found ID: " + std::to_string(id) + ")");
    }

    return {};
}

auto imu::Bno055Model::wait_for_device_ready(
    int max_attempts, std::chrono::milliseconds retry_interval) -> tl::expected<void, std::string> {
    if (max_attempts <= 0) {
        return tl::make_unexpected("max_attempts must be positive");
    }

    std::string last_error;

    for (int i = 1; i <= max_attempts; ++i) {
        const auto res = this->validate_id();
        if (res) {
            return {};
        }
        last_error = res.error();
        if (i < max_attempts) {
            rclcpp::sleep_for(retry_interval);
        }
    }
    return tl::make_unexpected(
        "device was not ready after " + std::to_string(max_attempts) + " attempts: " + last_error);
}

auto imu::Bno055Model::reset_device() -> tl::expected<void, std::string> {
    // コンフィグモードに移行（デフォルトでコンフィグモードだが念のため）
    auto res = this->write_reg(OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
    if (!res) {
        return tl::make_unexpected("Failed to set BNO055 to CONFIG mode: " + res.error());
    }
    rclcpp::sleep_for(20ms);

    // リセット
    res = this->write_reg(SYS_TRIGGER_ADDR, std::byte{0x20});
    if (!res) {
        return tl::make_unexpected("Failed to trigger BNO055 reset: " + res.error());
    }
    rclcpp::sleep_for(30ms);

    // BNO055が再起動するまで待機
    constexpr auto RESET_TIMEOUT = 1000ms;
    constexpr auto RESET_WAIT_INTERVAL = 10ms;
    constexpr auto RESET_VERIFY_ATTEMPTS = static_cast<int>(RESET_TIMEOUT / RESET_WAIT_INTERVAL);
    res = this->wait_for_device_ready(RESET_VERIFY_ATTEMPTS, RESET_WAIT_INTERVAL);
    if (!res) {
        return tl::make_unexpected(
            "BNO055 did not restart within timeout period (" +
            std::to_string(RESET_TIMEOUT.count()) + " ms): " + res.error());
    }

    rclcpp::sleep_for(50ms);
    return {};
}

auto imu::Bno055Model::configure_device() -> tl::expected<void, std::string> {
    // ノーマルパワーモードに設定
    auto res = this->write_reg(PWR_MODE_ADDR, POWER_MODE_NORMAL);
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

    // NDOFモードに設定
    res = this->write_reg(OPR_MODE_ADDR, OPERATION_MODE_NDOF);
    if (!res) {
        return tl::make_unexpected("Failed to set BNO055 to NDOF mode: " + res.error());
    }
    rclcpp::sleep_for(20ms);

    return {};
}

auto imu::Bno055Model::begin() -> tl::expected<void, std::string> {
    auto res = this->i2c->open();
    if (!res) {
        return tl::make_unexpected("Failed to open I2C bus for BNO055: " + res.error());
    }

    // 正しいデバイスであることを確認
    res = this->wait_for_device_ready(2, 100ms);
    if (!res) {
        return tl::make_unexpected("Failed to verify BNO055 device: " + res.error());
    }

    res = this->reset_device();
    if (!res) {
        return tl::make_unexpected("Failed to reset BNO055 device: " + res.error());
    }

    res = this->configure_device();
    if (!res) {
        return tl::make_unexpected("Failed to configure BNO055 device: " + res.error());
    }

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

    // rad/sに変換
    constexpr auto PI = boost::math::constants::pi<double>();
    constexpr auto GYRO_SCALE = (PI / 180.0) / 16.0;
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

    // BNO055内で重力加速度を除いてある線形加速度を使用
    constexpr auto LINEAR_ACCEL_SCALE = 1.0 / 100.0;
    const auto acceleration = state::imu::Acceleration{
        static_cast<double>(to_s16(&buffer[OFFSET_LINEAR_ACCEL + 0])) * LINEAR_ACCEL_SCALE,
        static_cast<double>(to_s16(&buffer[OFFSET_LINEAR_ACCEL + 2])) * LINEAR_ACCEL_SCALE,
        static_cast<double>(to_s16(&buffer[OFFSET_LINEAR_ACCEL + 4])) * LINEAR_ACCEL_SCALE};

    const auto temperature = state::imu::Temperature{std::to_integer<int8_t>(buffer[OFFSET_TEMP])};

    return ReadResult{quaternion, acceleration, angular_velocity, temperature};
}

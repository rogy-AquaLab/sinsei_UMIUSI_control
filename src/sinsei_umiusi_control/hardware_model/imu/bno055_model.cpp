#include "sinsei_umiusi_control/hardware_model/imu/bno055_model.hpp"

#include <array>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

using namespace sinsei_umiusi_control::hardware_model;

// ref: https://github.com/adafruit/Adafruit_BNO055/blob/1b1af09/Adafruit_BNO055.cpp

imu::Bno055Model::Bno055Model(std::unique_ptr<interface::I2c> i2c) : i2c(std::move(i2c)) {}

auto imu::Bno055Model::begin() -> tl::expected<void, std::string> {
    auto res = this->i2c->open(ADDRESS).map_error(interface::i2c_error_to_string);
    if (!res) {
        return tl::make_unexpected(
            "Failed to open I2C bus for BNO055: I2C open error: " + res.error());
    }

    // 正しいデバイスであることを確認
    auto id_opt_res = this->i2c->read_byte_data(CHIP_ID_ADDR)
                          .map_error(interface::i2c_error_to_string)
                          .map(std::to_integer<interface::I2c::Addr>);
    if (!id_opt_res || id_opt_res.value() != ID) {
        rclcpp::sleep_for(static_cast<std::chrono::nanoseconds>(1000ms));  // hold on for boot
        id_opt_res = this->i2c->read_byte_data(CHIP_ID_ADDR)
                         .map_error(interface::i2c_error_to_string)
                         .map(std::to_integer<interface::I2c::Addr>);
        if (!id_opt_res) {
            return tl::make_unexpected(
                "Failed to read CHIP_ID from BNO055: I2C read error: " + id_opt_res.error());
        }
        if (id_opt_res.value() != ID) {
            return tl::make_unexpected(
                "Failed to find BNO055 device (found ID: " + std::to_string(id_opt_res.value()) +
                ")");
        }
    }

    // コンフィグモードに移行（デフォルトでコンフィグモードだが念のため）
    res = this->i2c->write_byte_data(OPR_MODE_ADDR, std::byte{OPERATION_MODE_CONFIG})
              .map_error(interface::i2c_error_to_string);
    if (!res) {
        return tl::make_unexpected(
            "Failed to set BNO055 to CONFIG mode: I2C write error: " + res.error());
    }

    // リセット
    res = this->i2c->write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x20})
              .map_error(interface::i2c_error_to_string);
    if (!res) {
        return tl::make_unexpected(
            "Failed to trigger BNO055 reset: I2C write error: " + res.error());
    }

    // BNO055が再起動するまで待機
    rclcpp::sleep_for(static_cast<std::chrono::nanoseconds>(30ms));
    constexpr int TIMEOUT_MS = 1000;
    constexpr int WAIT_INTERVAL_MS = 10;
    bool timeout = true;
    for (int time = 0; time < TIMEOUT_MS; time += WAIT_INTERVAL_MS) {
        auto res = this->i2c->read_byte_data(CHIP_ID_ADDR)
                       .map_error(interface::i2c_error_to_string)
                       .map(std::to_integer<interface::I2c::Addr>);
        if (res && res.value() == ID) {  // 正常にBNO055が起動したことを確認
            timeout = false;
            break;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(WAIT_INTERVAL_MS));
    }
    if (timeout) {
        return tl::make_unexpected(
            "BNO055 did not restart within timeout period (" + std::to_string(TIMEOUT_MS) + " ms)");
    }
    rclcpp::sleep_for(static_cast<std::chrono::nanoseconds>(50ms));

    // ノーマルパワーモードに設定
    res = this->i2c->write_byte_data(PWR_MODE_ADDR, std::byte{POWER_MODE_NORMAL})
              .map_error(interface::i2c_error_to_string);
    if (!res) {
        return tl::make_unexpected(
            "Failed to set BNO055 to NORMAL power mode: I2C write error: " + res.error());
    }
    rclcpp::sleep_for(static_cast<std::chrono::nanoseconds>(10ms));

    res = this->i2c->write_byte_data(PAGE_ID_ADDR, std::byte{0x0})
              .map_error(interface::i2c_error_to_string);
    if (!res) {
        return tl::make_unexpected(
            "Failed to set BNO055 to PAGE 0: I2C write error: " + res.error());
    }

    res = this->i2c->write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x0})
              .map_error(interface::i2c_error_to_string);
    if (!res) {
        return tl::make_unexpected(
            "Failed to clear BNO055 SYS_TRIGGER: I2C write error: " + res.error());
    }
    rclcpp::sleep_for(static_cast<std::chrono::nanoseconds>(10ms));

    // NDOFモードに設定
    res = this->i2c->write_byte_data(OPR_MODE_ADDR, std::byte{OPERATION_MODE_NDOF})
              .map_error(interface::i2c_error_to_string);
    if (!res) {
        return tl::make_unexpected(
            "Failed to set BNO055 to NDOF mode: I2C write error: " + res.error());
    }
    rclcpp::sleep_for(static_cast<std::chrono::nanoseconds>(20ms));

    return {};
}

auto imu::Bno055Model::close() -> tl::expected<void, std::string> {
    auto res = this->i2c->close().map_error(interface::i2c_error_to_string);
    if (!res) {
        return tl::make_unexpected("Failed to close I2C bus: " + res.error());
    }
    return {};
}

auto imu::Bno055Model::read_measurements() -> tl::expected<Measurements, std::string> {
    static constexpr auto FRAME_START = GYRO_DATA_X_LSB_ADDR;
    static constexpr size_t FRAME_LENGTH = TEMP_ADDR - FRAME_START + 1;

    static constexpr size_t OFFSET_GYRO = 0;
    static constexpr size_t OFFSET_QUAT = QUATERNION_DATA_W_LSB_ADDR - FRAME_START;
    static constexpr size_t OFFSET_LINEAR_ACCEL = LINEAR_ACCEL_DATA_X_LSB_ADDR - FRAME_START;
    static constexpr size_t OFFSET_TEMP = TEMP_ADDR - FRAME_START;

    auto address_buffer =
        std::array<std::byte, 1>{std::byte{static_cast<uint8_t>(FRAME_START & 0xFF)}};
    auto read_buffer = std::array<std::byte, FRAME_LENGTH>{};

    const std::vector<interface::I2cMessage> messages = {
        {interface::I2cDirection::Write, address_buffer.data(), address_buffer.size()},
        {interface::I2cDirection::Read, read_buffer.data(), read_buffer.size()}};

    auto res = this->i2c->transfer(messages).map_error(interface::i2c_error_to_string);
    if (!res) {
        return tl::make_unexpected("I2C transfer error: " + res.error());
    }

    const auto read_s16 = [&read_buffer](size_t offset) -> int16_t {
        return to_s16(read_buffer[offset], read_buffer[offset + 1]);
    };

    const auto gyro_scale = this->get_scale(VectorType::Gyroscope);
    const auto angular_velocity = state::imu::AngularVelocity{
        static_cast<double>(read_s16(OFFSET_GYRO + 0)) * gyro_scale,
        static_cast<double>(read_s16(OFFSET_GYRO + 2)) * gyro_scale,
        static_cast<double>(read_s16(OFFSET_GYRO + 4)) * gyro_scale};

    constexpr double QUAT_SCALE = 1.0 / (1 << 14);
    const auto quaternion = state::imu::Quaternion{
        static_cast<double>(read_s16(OFFSET_QUAT + 0)) * QUAT_SCALE,
        static_cast<double>(read_s16(OFFSET_QUAT + 2)) * QUAT_SCALE,
        static_cast<double>(read_s16(OFFSET_QUAT + 4)) * QUAT_SCALE,
        static_cast<double>(read_s16(OFFSET_QUAT + 6)) * QUAT_SCALE};

    const auto accel_scale = this->get_scale(VectorType::LinearAccel);
    const auto acceleration = state::imu::Acceleration{
        static_cast<double>(read_s16(OFFSET_LINEAR_ACCEL + 0)) * accel_scale,
        static_cast<double>(read_s16(OFFSET_LINEAR_ACCEL + 2)) * accel_scale,
        static_cast<double>(read_s16(OFFSET_LINEAR_ACCEL + 4)) * accel_scale};

    const auto temp_raw = read_buffer[OFFSET_TEMP];
    // 取得した温度が「正しい温度 ± 128」となっている場合があるため補正する
    const auto fixed_temp = temp_raw & std::byte{0x7F};
    const auto temperature = state::imu::Temperature{static_cast<int8_t>(fixed_temp)};

    return Measurements{quaternion, acceleration, angular_velocity, temperature};
}

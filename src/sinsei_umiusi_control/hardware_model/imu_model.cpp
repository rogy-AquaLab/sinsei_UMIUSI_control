#include "sinsei_umiusi_control/hardware_model/imu_model.hpp"

#include <array>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

namespace sinsei_umiusi_control::hardware_model {

ImuModel::ImuModel(std::unique_ptr<interface::I2c> i2c) : i2c(std::move(i2c)) {}

auto ImuModel::on_init() -> tl::expected<void, std::string> {
    auto res = this->i2c->open();
    if (!res) {
        return tl::make_unexpected("Failed to open I2C bus: " + res.error());
    }

    // 正しいデバイスであることを確認
    auto read_id = [&]() -> tl::expected<uint8_t, std::string> {
        std::byte val;
        return this->i2c->read_reg(ADDRESS, CHIP_ID_ADDR, {&val, 1}).map([&val]() {
            return std::to_integer<uint8_t>(val);
        });
    };

    auto id_res = read_id();
    if (!id_res || id_res.value() != this->ID) {
        rclcpp::sleep_for(1000ms);  // hold on for boot
        id_res = read_id();
        if (!id_res) return tl::make_unexpected("Failed to read CHIP_ID: " + id_res.error());
        if (id_res.value() != this->ID) {
            return tl::make_unexpected(
                "BNO055 not found. Found ID: " + std::to_string(id_res.value()));
        }
    }

    // コンフィグモードに移行（デフォルトでコンフィグモードだが念のため）
    res = this->i2c->write_reg(ADDRESS, OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
    if (!res) {
        return tl::make_unexpected("Failed to set CONFIG mode: " + res.error());
    }

    // リセット
    res = this->i2c->write_reg(ADDRESS, SYS_TRIGGER_ADDR, std::byte{0x20});
    if (!res) {
        return tl::make_unexpected("Failed to trigger reset: " + res.error());
    }

    // BNO055が再起動するまで待機
    rclcpp::sleep_for(30ms);
    bool timeout = true;
    for (int i = 0; i < 100; ++i) {  // 10ms * 100 = 1000ms
        auto current_id = read_id();
        if (current_id && current_id.value() == this->ID) {
            timeout = false;
            break;
        }
        rclcpp::sleep_for(10ms);
    }
    if (timeout) return tl::make_unexpected("BNO055 restart timeout");

    rclcpp::sleep_for(50ms);

    // ノーマルパワーモードに設定
    this->i2c->write_reg(ADDRESS, PWR_MODE_ADDR, POWER_MODE_NORMAL);
    rclcpp::sleep_for(10ms);
    this->i2c->write_reg(ADDRESS, PAGE_ID_ADDR, std::byte{0x00});
    this->i2c->write_reg(ADDRESS, SYS_TRIGGER_ADDR, std::byte{0x00});
    rclcpp::sleep_for(10ms);

    // NDOFモードに設定
    res = this->i2c->write_reg(ADDRESS, OPR_MODE_ADDR, OPERATION_MODE_NDOF);
    if (!res) return tl::make_unexpected("Failed to set NDOF mode: " + res.error());

    rclcpp::sleep_for(20ms);
    return {};
}

auto ImuModel::on_destroy() -> tl::expected<void, std::string> {
    auto res = this->i2c->close();
    if (!res) {
        return tl::make_unexpected("Failed to close I2C bus: " + res.error());
    }
    return {};
}

auto ImuModel::on_read() -> tl::expected<
                             std::tuple<
                                 state::imu::Quaternion, state::imu::Acceleration,
                                 state::imu::AngularVelocity, state::imu::Temperature>,
                             std::string> {
    static constexpr auto FRAME_START = GYRO_DATA_X_LSB_ADDR.value;
    static constexpr size_t FRAME_LENGTH =
        std::to_integer<size_t>(TEMP_ADDR.value) - std::to_integer<size_t>(FRAME_START) + 1;

    auto read_buffer = std::array<std::byte, FRAME_LENGTH>{};

    auto res = this->i2c->read_reg(
        ADDRESS, interface::I2cRegisterAddr{FRAME_START}, {read_buffer.data(), read_buffer.size()});
    if (!res) {
        return tl::make_unexpected("BNO055 read error: " + res.error());
    }

    const auto read_s16 = [&read_buffer](size_t offset) -> int16_t {
        return to_s16(read_buffer[offset], read_buffer[offset + 1]);
    };

    const size_t OFFSET_GYRO = 0;
    const size_t OFFSET_QUAT = std::to_integer<size_t>(QUATERNION_DATA_W_LSB_ADDR.value) -
                               std::to_integer<size_t>(FRAME_START);
    const size_t OFFSET_LINEAR_ACCEL = std::to_integer<size_t>(LINEAR_ACCEL_DATA_X_LSB_ADDR.value) -
                                       std::to_integer<size_t>(FRAME_START);
    const size_t OFFSET_TEMP =
        std::to_integer<size_t>(TEMP_ADDR.value) - std::to_integer<size_t>(FRAME_START);

    const auto GYRO_SCALE = 1.0 / 16.0;
    const auto angular_velocity = state::imu::AngularVelocity{
        read_s16(OFFSET_GYRO + 0) * GYRO_SCALE, read_s16(OFFSET_GYRO + 2) * GYRO_SCALE,
        read_s16(OFFSET_GYRO + 4) * GYRO_SCALE};

    constexpr auto QUAT_SCALE = 1.0 / (1 << 14);
    const auto quaternion = state::imu::Quaternion{
        static_cast<double>(read_s16(OFFSET_QUAT + 0)) * QUAT_SCALE,
        static_cast<double>(read_s16(OFFSET_QUAT + 2)) * QUAT_SCALE,
        static_cast<double>(read_s16(OFFSET_QUAT + 4)) * QUAT_SCALE,
        static_cast<double>(read_s16(OFFSET_QUAT + 6)) * QUAT_SCALE};

    const auto LINEAR_ACCEL_SCALE = 1.0 / 100.0;
    const auto acceleration = state::imu::Acceleration{
        static_cast<double>(read_s16(OFFSET_LINEAR_ACCEL + 0)) * LINEAR_ACCEL_SCALE,
        static_cast<double>(read_s16(OFFSET_LINEAR_ACCEL + 2)) * LINEAR_ACCEL_SCALE,
        static_cast<double>(read_s16(OFFSET_LINEAR_ACCEL + 4)) * LINEAR_ACCEL_SCALE};

    const auto temp_raw = read_buffer[OFFSET_TEMP];
    // 取得した温度が「正しい温度 ± 128」となっている場合があるため補正する
    const auto fixed_temp = temp_raw & std::byte{0x7F};
    const auto temperature = state::imu::Temperature{static_cast<int8_t>(fixed_temp)};

    return std::make_tuple(quaternion, acceleration, angular_velocity, temperature);
}

}  // namespace sinsei_umiusi_control::hardware_model

#include "sinsei_umiusi_control/hardware_model/imu/bno055_model.hpp"

#include <array>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

namespace sinsei_umiusi_control::hardware_model::imu {

Bno055Model::Bno055Model(std::unique_ptr<interface::I2c> i2c) : i2c(std::move(i2c)) {}

auto Bno055Model::begin() -> tl::expected<void, std::string> {
    auto res = this->i2c->open();
    if (!res) {
        return tl::make_unexpected("Failed to open I2C bus for BNO055: " + res.error());
    }

    auto read_id = [&]() -> tl::expected<uint8_t, std::string> {
        std::byte value{};
        return this->i2c->read_reg(ADDRESS, CHIP_ID_ADDR, {&value, 1}).map([&value]() {
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

    res = this->i2c->write_reg(ADDRESS, OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
    if (!res) {
        return tl::make_unexpected("Failed to set BNO055 to CONFIG mode: " + res.error());
    }

    res = this->i2c->write_reg(ADDRESS, SYS_TRIGGER_ADDR, std::byte{0x20});
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

    res = this->i2c->write_reg(ADDRESS, PWR_MODE_ADDR, POWER_MODE_NORMAL);
    if (!res) {
        return tl::make_unexpected("Failed to set BNO055 to NORMAL power mode: " + res.error());
    }
    rclcpp::sleep_for(10ms);

    res = this->i2c->write_reg(ADDRESS, PAGE_ID_ADDR, std::byte{0x00});
    if (!res) {
        return tl::make_unexpected("Failed to set BNO055 to PAGE 0: " + res.error());
    }

    res = this->i2c->write_reg(ADDRESS, SYS_TRIGGER_ADDR, std::byte{0x00});
    if (!res) {
        return tl::make_unexpected("Failed to clear BNO055 SYS_TRIGGER: " + res.error());
    }
    rclcpp::sleep_for(10ms);

    res = this->i2c->write_reg(ADDRESS, OPR_MODE_ADDR, OPERATION_MODE_NDOF);
    if (!res) {
        return tl::make_unexpected("Failed to set BNO055 to NDOF mode: " + res.error());
    }
    rclcpp::sleep_for(20ms);

    return {};
}

auto Bno055Model::close() -> tl::expected<void, std::string> {
    auto res = this->i2c->close();
    if (!res) {
        return tl::make_unexpected("Failed to close I2C bus: " + res.error());
    }
    return {};
}

auto Bno055Model::get_temp() -> tl::expected<state::imu::Temperature, std::string> {
    std::byte value{};
    const auto res = this->i2c->read_reg(ADDRESS, TEMP_ADDR, {&value, 1});
    if (!res) {
        return tl::make_unexpected("I2C read error: " + res.error());
    }

    const auto fixed_temp = value & std::byte{0x7F};
    return state::imu::Temperature{static_cast<int8_t>(fixed_temp)};
}

auto Bno055Model::get_vector(VectorType type) -> tl::expected<Vector3, std::string> {
    auto buffer = std::array<std::byte, 6>{};
    const auto res = this->i2c->read_reg(ADDRESS, this->get_address(type), {buffer.data(), buffer.size()});
    if (!res) {
        return tl::make_unexpected("I2C read error: " + res.error());
    }

    const auto x_raw = to_s16(buffer[0], buffer[1]);
    const auto y_raw = to_s16(buffer[2], buffer[3]);
    const auto z_raw = to_s16(buffer[4], buffer[5]);
    const auto scale = this->get_scale(type);

    return Vector3{
        static_cast<double>(x_raw) * scale, static_cast<double>(y_raw) * scale,
        static_cast<double>(z_raw) * scale};
}

auto Bno055Model::get_quat() -> tl::expected<state::imu::Quaternion, std::string> {
    auto buffer = std::array<std::byte, 8>{};
    const auto res =
        this->i2c->read_reg(ADDRESS, QUATERNION_DATA_W_LSB_ADDR, {buffer.data(), buffer.size()});
    if (!res) {
        return tl::make_unexpected("I2C read error: " + res.error());
    }

    const auto w_raw = to_s16(buffer[0], buffer[1]);
    const auto x_raw = to_s16(buffer[2], buffer[3]);
    const auto y_raw = to_s16(buffer[4], buffer[5]);
    const auto z_raw = to_s16(buffer[6], buffer[7]);

    constexpr double SCALE = 1.0 / (1 << 14);
    return state::imu::Quaternion{
        static_cast<double>(w_raw) * SCALE, static_cast<double>(x_raw) * SCALE,
        static_cast<double>(y_raw) * SCALE, static_cast<double>(z_raw) * SCALE};
}

auto Bno055Model::get_acceleration() -> tl::expected<state::imu::Acceleration, std::string> {
    const auto res = this->get_vector(VectorType::LinearAccel);
    if (!res) {
        return tl::make_unexpected(res.error());
    }
    const auto [x, y, z] = res.value();
    return state::imu::Acceleration{x, y, z};
}

auto Bno055Model::get_angular_velocity()
    -> tl::expected<state::imu::AngularVelocity, std::string> {
    const auto res = this->get_vector(VectorType::Gyroscope);
    if (!res) {
        return tl::make_unexpected(res.error());
    }
    const auto [x, y, z] = res.value();
    return state::imu::AngularVelocity{x, y, z};
}

}  // namespace sinsei_umiusi_control::hardware_model::imu

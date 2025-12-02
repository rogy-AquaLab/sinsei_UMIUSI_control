
#include "sinsei_umiusi_control/hardware_model/imu_model.hpp"

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "sinsei_umiusi_control/state/imu.hpp"

using namespace std::chrono_literals;

using namespace sinsei_umiusi_control::hardware_model;

ImuModel::ImuModel(std::unique_ptr<interface::Gpio> gpio) : gpio(std::move(gpio)) {}

auto ImuModel::on_init() -> tl::expected<void, std::string> {
    auto res = this->gpio->i2c_open(ADDRESS).map_error(interface::gpio_error_to_string);
    if (!res) {
        return tl::make_unexpected(
            "Failed to open I2C bus for BNO055: I2C open error: " + res.error());
    }

    // 正しいデバイスであることを確認
    auto id_opt_res = this->gpio->i2c_read_byte_data(CHIP_ID_ADDR)
                          .map_error(interface::gpio_error_to_string)
                          .map(std::to_integer<interface::Gpio::Addr>);
    if (!id_opt_res || id_opt_res.value() != ID) {
        rclcpp::sleep_for(static_cast<std::chrono::nanoseconds>(1000ms));  // hold on for boot
        id_opt_res = this->gpio->i2c_read_byte_data(CHIP_ID_ADDR)
                         .map_error(interface::gpio_error_to_string)
                         .map(std::to_integer<interface::Gpio::Addr>);
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
    res = this->gpio->i2c_write_byte_data(OPR_MODE_ADDR, std::byte{OPERATION_MODE_CONFIG})
              .map_error(interface::gpio_error_to_string);
    if (!res) {
        return tl::make_unexpected(
            "Failed to set BNO055 to CONFIG mode: I2C write error: " + res.error());
    }

    // リセット
    res = this->gpio->i2c_write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x20})
              .map_error(interface::gpio_error_to_string);
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
        auto res = this->gpio->i2c_read_byte_data(CHIP_ID_ADDR)
                       .map_error(interface::gpio_error_to_string)
                       .map(std::to_integer<interface::Gpio::Addr>);
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
    res = this->gpio->i2c_write_byte_data(PWR_MODE_ADDR, std::byte{POWER_MODE_NORMAL})
              .map_error(interface::gpio_error_to_string);
    if (!res) {
        return tl::make_unexpected(
            "Failed to set BNO055 to NORMAL power mode: I2C write error: " + res.error());
    }
    rclcpp::sleep_for(static_cast<std::chrono::nanoseconds>(10ms));

    res = this->gpio->i2c_write_byte_data(PAGE_ID_ADDR, std::byte{0x0})
              .map_error(interface::gpio_error_to_string);
    if (!res) {
        return tl::make_unexpected(
            "Failed to set BNO055 to PAGE 0: I2C write error: " + res.error());
    }

    res = this->gpio->i2c_write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x0})
              .map_error(interface::gpio_error_to_string);
    if (!res) {
        return tl::make_unexpected(
            "Failed to clear BNO055 SYS_TRIGGER: I2C write error: " + res.error());
    }
    rclcpp::sleep_for(static_cast<std::chrono::nanoseconds>(10ms));

    // NDOFモードに設定
    res = this->gpio->i2c_write_byte_data(OPR_MODE_ADDR, std::byte{OPERATION_MODE_NDOF})
              .map_error(interface::gpio_error_to_string);
    if (!res) {
        return tl::make_unexpected(
            "Failed to set BNO055 to NDOF mode: I2C write error: " + res.error());
    }
    rclcpp::sleep_for(static_cast<std::chrono::nanoseconds>(20ms));

    return {};
}

auto ImuModel::on_destroy() -> tl::expected<void, std::string> {
    auto res = this->gpio->i2c_close().map_error(interface::gpio_error_to_string);
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
    // 通信回数削減のため一括でデータを取得する
    std::array<std::byte, 45> buf;

    // const auto res = this->gpio->i2c_read_block_data(START_ADDR, buf.data(), buf.size())
    //                      .map_error(interface::gpio_error_to_string);

    // if (!res) {
    //     return tl::make_unexpected("I2C read error: " + res.error());
    // }

    // レジスタのアドレスをセット
    auto wres = this->gpio->i2c_write_byte(static_cast<std::byte>(START_ADDR))
                    .map_error(interface::gpio_error_to_string);
    if (!wres) {
        return tl::make_unexpected("I2C write error: " + wres.error());
    }

    // データを一括取得
    auto rres = this->gpio->i2c_read_device(buf.data(), buf.size())
                    .map_error(interface::gpio_error_to_string);
    if (!rres) {
        return tl::make_unexpected("I2C read error: " + rres.error());
    }

    constexpr auto ACC_SCALE = 1.0 / 100.0;
    constexpr auto GYRO_SCALE = 1.0 / 16.0;
    constexpr auto QUAT_SCALE = 1.0 / (1 << 14);

    // Skip Raw Accelaration (0-5) and Magnetometer (6-11)

    // Gyroscope (12–17)
    const auto angular_velocity = state::imu::AngularVelocity{
        to_s16(buf, 12) * GYRO_SCALE,  // x
        to_s16(buf, 14) * GYRO_SCALE,  // y
        to_s16(buf, 16) * GYRO_SCALE   // z
    };

    // Skip Euler angles (18-23)

    // Quaternion (24–31)
    const auto quat = state::imu::Quaternion{
        to_s16(buf, 24) * QUAT_SCALE,  // w
        to_s16(buf, 26) * QUAT_SCALE,  // x
        to_s16(buf, 28) * QUAT_SCALE,  // y
        to_s16(buf, 30) * QUAT_SCALE   // z
    };

    // Linear acceleration (32–37)
    const auto linear_acceleration = state::imu::Acceleration{
        to_s16(buf, 32) * ACC_SCALE,  // x
        to_s16(buf, 34) * ACC_SCALE,  // y
        to_s16(buf, 36) * ACC_SCALE   // z
    };

    // Skip Gravity (38-43)

    // -- Temperature (44)
    const auto temp_raw = buf[44];
    // 取得した温度が「正しい温度 ± 128」となっている場合があるため補正する
    const auto fixed_temp = temp_raw & std::byte{0x7F};
    const auto temp = state::imu::Temperature{static_cast<int8_t>(fixed_temp)};

    return std::make_tuple(quat, linear_acceleration, angular_velocity, temp);
}

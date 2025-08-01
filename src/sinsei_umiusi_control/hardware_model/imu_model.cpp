#include "sinsei_umiusi_control/hardware_model/imu_model.hpp"

#include <chrono>

#include "sinsei_umiusi_control/state/imu.hpp"

using namespace std::chrono_literals;

using namespace sinsei_umiusi_control::hardware_model;

// ref: https://github.com/adafruit/Adafruit_BNO055/blob/1b1af09/Adafruit_BNO055.cpp

ImuModel::ImuModel(std::unique_ptr<interface::Gpio> gpio) : gpio(std::move(gpio)) {}

auto ImuModel::on_init() -> tl::expected<void, std::string> {
    auto res = this->gpio->i2c_open(ADDRESS).map_error(interface::gpio_error_to_string);
    if (!res) {
        return tl::unexpected<std::string>(
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
            return tl::unexpected<std::string>(
                "Failed to read CHIP_ID from BNO055: I2C read error: " + id_opt_res.error());
        }
        if (id_opt_res.value() != ID) {
            return tl::unexpected<std::string>(
                "Failed to find BNO055 device (found ID: " + std::to_string(id_opt_res.value()) +
                ")");
        }
    }

    // コンフィグモードに移行（デフォルトでコンフィグモードだが念のため）
    res = this->gpio->i2c_write_byte_data(OPR_MODE_ADDR, std::byte{OPERATION_MODE_CONFIG})
              .map_error(interface::gpio_error_to_string);
    if (!res) {
        return tl::unexpected<std::string>(
            "Failed to set BNO055 to CONFIG mode: I2C write error: " + res.error());
    }

    // リセット
    res = this->gpio->i2c_write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x20})
              .map_error(interface::gpio_error_to_string);
    if (!res) {
        return tl::unexpected<std::string>(
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
        return tl::unexpected<std::string>(
            "BNO055 did not restart within timeout period (" + std::to_string(TIMEOUT_MS) + " ms)");
    }
    rclcpp::sleep_for(static_cast<std::chrono::nanoseconds>(50ms));

    // ノーマルパワーモードに設定
    res = this->gpio->i2c_write_byte_data(PWR_MODE_ADDR, std::byte{POWER_MODE_NORMAL})
              .map_error(interface::gpio_error_to_string);
    if (!res) {
        return tl::unexpected<std::string>(
            "Failed to set BNO055 to NORMAL power mode: I2C write error: " + res.error());
    }
    rclcpp::sleep_for(static_cast<std::chrono::nanoseconds>(10ms));

    res = this->gpio->i2c_write_byte_data(PAGE_ID_ADDR, std::byte{0x0})
              .map_error(interface::gpio_error_to_string);
    if (!res) {
        return tl::unexpected<std::string>(
            "Failed to set BNO055 to PAGE 0: I2C write error: " + res.error());
    }

    res = this->gpio->i2c_write_byte_data(SYS_TRIGGER_ADDR, std::byte{0x0})
              .map_error(interface::gpio_error_to_string);
    if (!res) {
        return tl::unexpected<std::string>(
            "Failed to clear BNO055 SYS_TRIGGER: I2C write error: " + res.error());
    }
    rclcpp::sleep_for(static_cast<std::chrono::nanoseconds>(10ms));

    // NDOFモードに設定
    res = this->gpio->i2c_write_byte_data(OPR_MODE_ADDR, std::byte{OPERATION_MODE_NDOF})
              .map_error(interface::gpio_error_to_string);
    if (!res) {
        return tl::unexpected<std::string>(
            "Failed to set BNO055 to NDOF mode: I2C write error: " + res.error());
    }
    rclcpp::sleep_for(static_cast<std::chrono::nanoseconds>(20ms));

    return {};
}

auto ImuModel::on_read()
    -> tl::expected<
        std::tuple<state::imu::Quaternion, state::imu::Velocity, state::imu::Temperature>,
        std::string> {
    const auto quaternion_res = this->read_quat();
    if (!quaternion_res) {
        return tl::unexpected<std::string>(
            "Failed to read quaternion data from BNO055: " + quaternion_res.error());
    }

    // FIXME: dummy
    const state::imu::Velocity velocity{0.0, 0.0, 0.0};

    const auto temp_raw_res =
        this->gpio->i2c_read_byte_data(TEMP_ADDR).map_error(interface::gpio_error_to_string);
    if (!temp_raw_res) {
        return tl::unexpected<std::string>(
            "Failed to read temperature data from BNO055: I2C read error: " + temp_raw_res.error());
    }
    const auto temp_raw = temp_raw_res.value();
    // 取得した温度が「正しい温度 ± 128」となっている場合があるため補正する
    const auto fixed_temp = temp_raw & std::byte{0x7F};
    const state::imu::Temperature temperature{static_cast<int8_t>(fixed_temp)};

    return std::make_tuple(quaternion_res.value(), velocity, temperature);
}

auto ImuModel::read_quat() -> tl::expected<state::imu::Quaternion, std::string> {
    // ref: https://github.com/adafruit/Adafruit_BNO055/blob/1b1af09/Adafruit_BNO055.cpp#L466

    auto buffer = std::array<std::byte, 8>{};

    for (int i = 0; i < 8; ++i) {
        auto byte_opt_res = this->gpio->i2c_read_byte_data(QUATERNION_DATA_W_LSB_ADDR + i)
                                .map_error(interface::gpio_error_to_string);
        if (!byte_opt_res) {
            return tl::unexpected<std::string>("I2C read error: " + byte_opt_res.error());
        }
        buffer[i] = byte_opt_res.value();
    }

    const auto w = std::to_integer<int16_t>(buffer[0]) | (std::to_integer<int16_t>(buffer[1]) << 8);
    const auto x = std::to_integer<int16_t>(buffer[2]) | (std::to_integer<int16_t>(buffer[3]) << 8);
    const auto y = std::to_integer<int16_t>(buffer[4]) | (std::to_integer<int16_t>(buffer[5]) << 8);
    const auto z = std::to_integer<int16_t>(buffer[6]) | (std::to_integer<int16_t>(buffer[7]) << 8);

    constexpr auto SCALE = 1.0 / (1 << 14);

    const state::imu::Quaternion quaternion{
        static_cast<double>(x) * SCALE, static_cast<double>(y) * SCALE,
        static_cast<double>(z) * SCALE, static_cast<double>(w) * SCALE};
    return quaternion;
}

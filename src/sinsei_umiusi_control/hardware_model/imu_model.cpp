#include "sinsei_umiusi_control/hardware_model/imu_model.hpp"

namespace suc = sinsei_umiusi_control;
namespace suchm = suc::hardware_model;

// ref: https://github.com/adafruit/Adafruit_BNO055/blob/1b1af09/Adafruit_BNO055.cpp

suchm::ImuModel::ImuModel(std::unique_ptr<suc::util::Gpio> gpio) : gpio(std::move(gpio)) {}

auto suchm::ImuModel::begin() -> tl::expected<void, std::string> {
    // TODO: エラーを捕捉する
    if (!this->gpio->i2c_open(ADDRESS)) {
        return tl::unexpected<std::string>("Failed to open I2C bus for BNO055");
    }

    // 正しいデバイスであることを確認
    // TODO: エラーを捕捉する
    auto id_opt = this->gpio->i2c_read_byte_data(CHIP_ID_ADDR);
    if (!id_opt || id_opt.value() != ID) {
        rclcpp::sleep_for(std::chrono::milliseconds(1000));  // hold on for boot
        // TODO: エラーを捕捉する
        auto id_opt = this->gpio->i2c_read_byte_data(CHIP_ID_ADDR);
        if (!id_opt || id_opt.value() != ID) {
            return tl::unexpected<std::string>("Failed to find BNO055 device");
        }
    }

    // コンフィグモードに移行（デフォルトでコンフィグモードだが念のため）
    // TODO: エラーを捕捉する
    this->gpio->i2c_write_byte_data(OPR_MODE_ADDR, OPERATION_MODE_CONFIG);

    // リセット
    // TODO: エラーを捕捉する
    this->gpio->i2c_write_byte_data(SYS_TRIGGER_ADDR, 0x20);

    rclcpp::sleep_for(std::chrono::milliseconds(30));
    int timeout_ms = 1000;
    constexpr int WAIT_INTERVAL_MS = 10;
    // TODO: エラーを捕捉する
    while (this->gpio->i2c_read_byte_data(CHIP_ID_ADDR) != ID && timeout_ms > 0) {
        if (timeout_ms <= 0) {
            return tl::unexpected<std::string>("BNO055 reset timeout");
        }
        rclcpp::sleep_for(std::chrono::milliseconds(WAIT_INTERVAL_MS));
        timeout_ms -= WAIT_INTERVAL_MS;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(50));

    // ノーマルパワーモードに設定
    // TODO: エラーを捕捉する
    this->gpio->i2c_write_byte_data(PWR_MODE_ADDR, POWER_MODE_NORMAL);
    rclcpp::sleep_for(std::chrono::milliseconds(10));

    // TODO: エラーを捕捉する
    this->gpio->i2c_write_byte_data(PAGE_ID_ADDR, 0x0);

    // TODO: エラーを捕捉する
    this->gpio->i2c_write_byte_data(SYS_TRIGGER_ADDR, 0x0);
    rclcpp::sleep_for(std::chrono::milliseconds(10));

    // NDOFモードに設定
    // TODO: エラーを捕捉する
    this->gpio->i2c_write_byte_data(OPR_MODE_ADDR, OPERATION_MODE_NDOF);
    rclcpp::sleep_for(std::chrono::milliseconds(20));

    return {};
}

auto suchm::ImuModel::on_read()
    -> tl::expected<
        std::tuple<
            suc::state::imu::Orientation, suc::state::imu::Velocity, suc::state::imu::Temperature>,
        std::string> {
    // TODO: エラーを捕捉する
    const auto orientation = this->read_orientation().value_or(state::imu::Orientation{});
    const state::imu::Velocity velocity{0.0, 0.0, 0.0};

    // TODO: エラーを捕捉する
    const auto temp_raw = this->gpio->i2c_read_byte_data(TEMP_ADDR);
    const suc::state::imu::Temperature temperature{static_cast<int8_t>(temp_raw.value_or(0))};

    return std::make_tuple(orientation, velocity, temperature);
}

auto suchm::ImuModel::read_orientation() -> tl::expected<state::imu::Orientation, std::string> {
    // ref: https://github.com/adafruit/Adafruit_BNO055/blob/1b1af09/Adafruit_BNO055.cpp#L401

    std::array<uint8_t, 6> buffer{};

    for (int i = 0; i < 6; ++i) {
        // TODO: エラーを捕捉する
        auto byte_opt = this->gpio->i2c_read_byte_data(EULER_H_LSB_ADDR + i);
        if (!byte_opt) {
            return tl::unexpected<std::string>("Failed to read orientation data from BNO055");
        }
        buffer[i] = byte_opt.value();
    }

    const int16_t x = static_cast<int16_t>(
        static_cast<int16_t>(buffer[0]) | (static_cast<int16_t>(buffer[1]) << 8));
    const int16_t y = static_cast<int16_t>(
        static_cast<int16_t>(buffer[2]) | (static_cast<int16_t>(buffer[3]) << 8));
    const int16_t z = static_cast<int16_t>(
        static_cast<int16_t>(buffer[4]) | (static_cast<int16_t>(buffer[5]) << 8));

    const state::imu::Orientation orientation{
        static_cast<double>(x) / 16.0, static_cast<double>(y) / 16.0,
        static_cast<double>(z) / 16.0};

    return orientation;
}

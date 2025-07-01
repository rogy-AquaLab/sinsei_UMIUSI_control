#include "sinsei_umiusi_control/hardware_model/imu_model.hpp"

#include <memory>

#include "sinsei_umiusi_control/util/gpio.hpp"

namespace suc = sinsei_umiusi_control;
namespace suchm = suc::hardware_model;

// ref: https://github.com/adafruit/Adafruit_BNO055/blob/1b1af09/Adafruit_BNO055.cpp

suchm::ImuModel::ImuModel(std::unique_ptr<suc::util::Gpio> gpio_) : gpio(std::move(gpio_)) {}

auto suchm::ImuModel::begin() -> tl::expected<void, std::string> {
    if (!gpio->i2c_open(ADDRESS)) {
        return tl::unexpected<std::string>("Failed to open I2C bus for BNO055");
    }

    // 正しいデバイスであることを確認
    auto id_opt = gpio->i2c_read_byte_data(CHIP_ID_ADDR);
    if (!id_opt || id_opt.value() != ID) {
        rclcpp::sleep_for(std::chrono::milliseconds(1000));  // hold on for boot
        auto id_opt = gpio->i2c_read_byte_data(CHIP_ID_ADDR);
        if (!id_opt || id_opt.value() != ID) {
            return tl::unexpected<std::string>("Failed to find BNO055 device");
        }
    }

    // コンフィグモードに移行（デフォルトでコンフィグモードだが念のため）
    write_reg(OPR_MODE_ADDR, OPERATION_MODE_CONFIG);

    // リセット
    write_reg(SYS_TRIGGER_ADDR, 0x20);

    rclcpp::sleep_for(std::chrono::milliseconds(30));
    int timeout = 1000;  // in ms
    while (gpio->i2c_read_byte_data(CHIP_ID_ADDR) != ID && timeout > 0) {
        int wait_time = 10;  // in ms
        rclcpp::sleep_for(std::chrono::milliseconds(wait_time));
        timeout -= wait_time;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(50));

    // ノーマルパワーモードに設定
    write_reg(PWR_MODE_ADDR, POWER_MODE_NORMAL);
    rclcpp::sleep_for(std::chrono::milliseconds(10));

    write_reg(PAGE_ID_ADDR, 0);

    write_reg(SYS_TRIGGER_ADDR, 0x0);
    rclcpp::sleep_for(std::chrono::milliseconds(10));

    // NDOFモードに設定
    write_reg(OPR_MODE_ADDR, OPERATION_MODE_NDOF);
    rclcpp::sleep_for(std::chrono::milliseconds(20));

    return {};
}

auto suchm::ImuModel::on_read(
    state::imu::Orientation & orientation, state::imu::Velocity & velocity,
    state::imu::Temperature & temperature) -> void {
    read_orientation(orientation);

    velocity.x = 0.0;
    velocity.y = 0.0;
    velocity.z = 0.0;

    auto temp_opt = gpio->i2c_read_byte_data(TEMP_ADDR);
    if (temp_opt) {
        temperature.value = static_cast<int8_t>(temp_opt.value());
    } else {
        temperature.value = 0;
    }
}

bool suchm::ImuModel::read_orientation(state::imu::Orientation & orientation) {
    std::vector<uint8_t> data(6);
    for (int i = 0; i < 6; ++i) {
        auto byte_opt = gpio->i2c_read_byte_data(EULER_H_LSB_ADDR + i);
        if (!byte_opt) return false;
        data[i] = byte_opt.value();
    }

    int roll = (data[3] << 8) | data[2];
    int pitch = (data[5] << 8) | data[4];
    int heading = (data[1] << 8) | data[0];

    orientation.x = static_cast<double>(roll) / 16.0f;
    orientation.y = static_cast<double>(pitch) / 16.0f;
    orientation.z = static_cast<double>(heading) / 16.0f;
    return true;
}

void suchm::ImuModel::write_reg(uint8_t reg, uint8_t value) {
    gpio->i2c_write_byte_data(reg, value);
    rclcpp::sleep_for(std::chrono::milliseconds(1));
}
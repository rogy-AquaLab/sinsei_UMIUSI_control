#include "sinsei_umiusi_control/hardware_model/imu_model.hpp"

#include <memory>

#include "sinsei_umiusi_control/util/gpio.hpp"

namespace suc = sinsei_umiusi_control;
namespace suchm = suc::hardware_model;

suchm::ImuModel::ImuModel(std::unique_ptr<suc::util::Gpio> gpio_, rclcpp::Clock & clock)
: gpio(std::move(gpio_)) {
    if (!gpio->i2c_open(ADDRESS)) return;

    auto deadline = clock.now() + rclcpp::Duration::from_seconds(1.0);

    // チップID確認（最大1秒）
    while (clock.now() < deadline) {
        auto id_opt = gpio->i2c_read_byte_data(CHIP_ID_ADDR);
        if (id_opt && id_opt.value() == 0xA0) break;
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }

    write_reg(OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
    write_reg(SYS_TRIGGER_ADDR, 0x20);  // reset

    deadline = clock.now() + rclcpp::Duration::from_seconds(1.0);
    while (clock.now() < deadline) {
        auto id_opt = gpio->i2c_read_byte_data(CHIP_ID_ADDR);
        if (id_opt && id_opt.value() == 0xA0) break;
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }

    write_reg(PWR_MODE_ADDR, POWER_MODE_NORMAL);
    write_reg(PAGE_ID_ADDR, 0);
    write_reg(OPR_MODE_ADDR, OPERATION_MODE_IMUPLUS);
    rclcpp::sleep_for(std::chrono::milliseconds(20));
}

auto suchm::ImuModel::on_read() -> sinsei_umiusi_control::state::imu::ImuState {
    state::imu::ImuState state;

    float h, r, p;
    if (read_euler(h, r, p)) {
        state.orientation.x = h;
        state.orientation.y = r;
        state.orientation.z = p;
    } else {
        state.orientation.x = 0.0;
        state.orientation.y = 0.0;
        state.orientation.z = 0.0;
    }

    state.velocity.x = 0.0;
    state.velocity.y = 0.0;
    state.velocity.z = 0.0;

    auto temp_opt = gpio->i2c_read_byte_data(BNO055_TEMP_ADDR);
    if (temp_opt) {
        state.temperature.value = static_cast<int8_t>(temp_opt.value());
    } else {
        state.temperature.value = 0;
    }
    return state;
}

bool suchm::ImuModel::read_euler(float & h, float & r, float & p) {
    std::vector<uint8_t> data(6);
    for (int i = 0; i < 6; ++i) {
        auto byte_opt = gpio->i2c_read_byte_data(EULER_H_LSB_ADDR + i);
        if (!byte_opt) return false;
        data[i] = byte_opt.value();
    }

    int heading = (data[1] << 8) | data[0];
    int roll = (data[3] << 8) | data[2];
    int pitch = (data[5] << 8) | data[4];

    h = heading / 16.0f;
    r = roll / 16.0f;
    p = pitch / 16.0f;
    return true;
}

void suchm::ImuModel::write_reg(uint8_t reg, uint8_t value) {
    gpio->i2c_write_byte_data(reg, value);
    rclcpp::sleep_for(std::chrono::milliseconds(1));
}
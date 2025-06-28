#include "sinsei_umiusi_control/util/pigpio.hpp"

#include <pigpiod_if2.h>

namespace suc_util = sinsei_umiusi_control::util;

suc_util::Pigpio::Pigpio(int pin_number) {
    this->pin_number = pin_number;
    this->pi = pigpio_start(NULL, NULL);
    set_mode(pi, pin_number, PI_OUTPUT);
}

suc_util::Pigpio::~Pigpio() { pigpio_stop(pi); }

auto suc_util::Pigpio::write_digital(bool enabled) -> GpioResult {
    gpio_write(pi, this->pin_number, enabled ? 1 : 0);
    return GpioResult::Success;
}

auto suc_util::Pigpio::write_pwm() -> GpioResult {
    // TODO: PWMの処理を実装する
    return GpioResult::Error;
}

auto suc_util::Pigpio::i2c_open(int address) -> bool {
    i2c_address = address;
    i2c_handle = ::i2c_open(pi, I2C_BUS, i2c_address, 0);
    return (i2c_handle >= 0);
}

auto suc_util::Pigpio::i2c_close() -> void {
    if (i2c_handle >= 0) {
        ::i2c_close(pi, i2c_handle);
        i2c_handle = -1;
    }
}

auto suc_util::Pigpio::i2c_write_byte(uint8_t value) -> bool {
    if (i2c_handle < 0) return false;
    return ::i2c_write_byte(pi, i2c_handle, value) == 0;
}

auto suc_util::Pigpio::i2c_read_byte() -> std::optional<uint8_t> {
    if (i2c_handle < 0) return std::nullopt;
    int result = ::i2c_read_byte(pi, i2c_handle);
    if (result < 0) return std::nullopt;
    return static_cast<uint8_t>(result);
}

auto suc_util::Pigpio::i2c_write_byte_data(uint8_t reg, uint8_t value) -> bool {
    if (i2c_handle < 0) return false;
    return ::i2c_write_byte_data(pi, i2c_handle, reg, value) == 0;
}

auto suc_util::Pigpio::i2c_read_byte_data(uint8_t reg) -> std::optional<uint8_t> {
    if (i2c_handle < 0) return std::nullopt;
    int result = ::i2c_read_byte_data(pi, i2c_handle, reg);
    if (result < 0) return std::nullopt;
    return static_cast<uint8_t>(result);
}
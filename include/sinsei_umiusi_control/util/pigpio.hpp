#ifndef SINSEI_UMIUSI_CONTROL_UTIL_PIGPIO_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_PIGPIO_HPP

#include <pigpiod_if2.h>

#include <optional>

#include "sinsei_umiusi_control/util/gpio.hpp"

namespace sinsei_umiusi_control::util {

class Pigpio : public Gpio {
  private:
    int pi;

    int pin_number;

    static constexpr int I2C_BUS = 1;
    int i2c_handle = -1;
    int i2c_address;

  public:
    Pigpio(int pin_number);
    ~Pigpio();

    auto write_digital(bool enabled) -> GpioResult override;
    auto write_pwm() -> GpioResult override;

    auto i2c_open(int address) -> bool override;
    auto i2c_close() -> void override;
    auto i2c_write_byte(uint8_t value) -> bool override;
    auto i2c_read_byte() -> std::optional<uint8_t> override;
    auto i2c_write_byte_data(uint8_t reg, uint8_t value) -> bool override;
    auto i2c_read_byte_data(uint8_t reg) -> std::optional<uint8_t> override;
};

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_PIGPIO_HPP
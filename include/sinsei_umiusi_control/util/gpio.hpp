#ifndef SINSEI_UMIUSI_CONTROL_UTIL_GPIO_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_GPIO_HPP

#include <cstdint>
#include <optional>

namespace sinsei_umiusi_control::util {

enum class GpioResult {
    Success,
    Error,
};

class Gpio {
  public:
    Gpio() = default;
    virtual ~Gpio() = default;
    virtual auto write_digital(bool enabled) -> GpioResult = 0;
    virtual auto write_pwm() -> GpioResult = 0;

    virtual auto i2c_open(int address) -> bool = 0;
    virtual auto i2c_close() -> void = 0;
    virtual auto i2c_write_byte(uint8_t value) -> bool = 0;
    virtual auto i2c_read_byte() -> std::optional<uint8_t> = 0;
    virtual auto i2c_write_byte_data(uint8_t reg, uint8_t value) -> bool = 0;
    virtual auto i2c_read_byte_data(uint8_t reg) -> std::optional<uint8_t> = 0;
};

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_GPIO_HPP
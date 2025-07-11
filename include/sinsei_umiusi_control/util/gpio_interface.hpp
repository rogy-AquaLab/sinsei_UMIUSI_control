#ifndef SINSEI_UMIUSI_CONTROL_UTIL_GPIO_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_GPIO_HPP

#include <cstdint>
#include <rcpputils/tl_expected/expected.hpp>

namespace sinsei_umiusi_control::util {

enum class GpioError {
    NotPermitted,
    BadGpio,
    BadLevel,
    BadFlags,
    BadHandle,
    BadParameter,
    NoHandle,
    I2cNotOpen,
    I2cBadBus,
    I2CBadAddress,
    I2cOpenFailed,
    I2cWriteFailed,
    I2cReadFailed,
    UnknownError,
};

class GpioInterface {
  public:
    GpioInterface() = default;
    virtual ~GpioInterface() = default;

    virtual auto set_mode_output(std::vector<uint8_t> pins) -> tl::expected<void, GpioError> = 0;
    virtual auto set_mode_input(std::vector<uint8_t> pins) -> tl::expected<void, GpioError> = 0;
    virtual auto write_digital(uint8_t pin, bool enabled) -> tl::expected<void, GpioError> = 0;
    virtual auto write_pwm() -> tl::expected<void, GpioError> = 0;

    virtual auto i2c_open(int address) -> tl::expected<void, GpioError> = 0;
    virtual auto i2c_close() -> tl::expected<void, GpioError> = 0;
    virtual auto i2c_write_byte(uint8_t value) -> tl::expected<void, GpioError> = 0;
    virtual auto i2c_read_byte() -> tl::expected<uint8_t, GpioError> = 0;
    virtual auto i2c_write_byte_data(uint8_t reg, uint8_t value)
        -> tl::expected<void, GpioError> = 0;
    virtual auto i2c_read_byte_data(uint8_t reg) -> tl::expected<uint8_t, GpioError> = 0;
};

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_GPIO_HPP
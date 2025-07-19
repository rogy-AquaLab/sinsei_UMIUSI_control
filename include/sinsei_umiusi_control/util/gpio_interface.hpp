#ifndef SINSEI_UMIUSI_CONTROL_UTIL_GPIO_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_GPIO_HPP

#include <cstddef>
#include <cstdint>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>

namespace sinsei_umiusi_control::util {

using GpioPin = uint8_t;

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

inline auto gpio_error_to_string(GpioError error) -> std::string {
    switch (error) {
        case GpioError::NotPermitted:
            return "Not permitted";
        case GpioError::BadGpio:
            return "Bad GPIO pin specified";
        case GpioError::BadLevel:
            return "Bad level specified";
        case GpioError::BadFlags:
            return "Bad flags specified";
        case GpioError::BadHandle:
            return "Bad handle specified";
        case GpioError::BadParameter:
            return "Bad parameter specified";
        case GpioError::NoHandle:
            return "No handle available";
        case GpioError::I2cNotOpen:
            return "I2C not open";
        case GpioError::I2cBadBus:
            return "I2C bad bus";
        case GpioError::I2CBadAddress:
            return "I2C bad address";
        case GpioError::I2cOpenFailed:
            return "I2C open failed";
        case GpioError::I2cWriteFailed:
            return "I2C write failed";
        case GpioError::I2cReadFailed:
            return "I2C read failed";
        default:
            return "Unknown error";
    }
}

class GpioInterface {
  public:
    GpioInterface() = default;
    virtual ~GpioInterface() = default;

    virtual auto set_mode_output(std::vector<GpioPin> pins) -> tl::expected<void, GpioError> = 0;
    virtual auto set_mode_input(std::vector<GpioPin> pins) -> tl::expected<void, GpioError> = 0;
    virtual auto write_digital(GpioPin pin, bool enabled) -> tl::expected<void, GpioError> = 0;
    virtual auto write_pwm() -> tl::expected<void, GpioError> = 0;

    virtual auto i2c_open(uint32_t address) -> tl::expected<void, GpioError> = 0;
    virtual auto i2c_close() -> tl::expected<void, GpioError> = 0;
    virtual auto i2c_write_byte(std::byte value) -> tl::expected<void, GpioError> = 0;
    virtual auto i2c_read_byte() -> tl::expected<std::byte, GpioError> = 0;
    virtual auto i2c_write_byte_data(uint32_t reg, std::byte value)
        -> tl::expected<void, GpioError> = 0;
    virtual auto i2c_read_byte_data(uint32_t reg) -> tl::expected<std::byte, GpioError> = 0;
};

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_GPIO_HPP
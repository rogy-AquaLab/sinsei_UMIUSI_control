#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_INTERFACE_GPIO_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_INTERFACE_GPIO_HPP

#include <cstddef>
#include <cstdint>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>

namespace sinsei_umiusi_control::hardware_model::interface {

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

inline auto gpio_error_to_string(const GpioError & error) -> std::string {
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

class Gpio {
  public:
    using Pin = uint32_t;   // GPIO pin number
    using Addr = uint32_t;  // I2C address / register address
    using Error = GpioError;

    Gpio() = default;
    virtual ~Gpio() = default;

    virtual auto set_mode_output(const std::vector<Pin> & pins) -> tl::expected<void, Error> = 0;
    virtual auto set_mode_input(const std::vector<Pin> & pins) -> tl::expected<void, Error> = 0;
    virtual auto write_digital(const Pin & pin, bool && enabled) -> tl::expected<void, Error> = 0;
    virtual auto write_pwm() -> tl::expected<void, Error> = 0;

    virtual auto i2c_open(const Addr & address) -> tl::expected<void, Error> = 0;
    virtual auto i2c_close() -> tl::expected<void, Error> = 0;
    virtual auto i2c_write_byte(std::byte && value) -> tl::expected<void, Error> = 0;
    virtual auto i2c_read_byte() const -> tl::expected<std::byte, Error> = 0;
    virtual auto i2c_write_byte_data(const Addr & reg, std::byte && value)
        -> tl::expected<void, Error> = 0;
    virtual auto i2c_read_byte_data(const Addr & reg) const -> tl::expected<std::byte, Error> = 0;
};

}  // namespace sinsei_umiusi_control::hardware_model::interface

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_INTERFACE_GPIO_HPP
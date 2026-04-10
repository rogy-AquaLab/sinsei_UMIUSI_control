#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_INTERFACE_GPIO_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_INTERFACE_GPIO_HPP

#include <cstdint>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>
#include <vector>

namespace sinsei_umiusi_control::hardware_model::interface {

enum class GpioError {
    NotPermitted,
    BadGpio,
    BadLevel,
    BadFlags,
    BadHandle,
    BadParameter,
    BadPulsewidth,
    NoHandle,
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
        case GpioError::BadPulsewidth:
            return "Bad pulsewidth specified";
        case GpioError::NoHandle:
            return "No handle available";
        default:
            return "Unknown error";
    }
}

class Gpio {
  public:
    using Pin = uint32_t;         // GPIO pin number
    using PulseWidth = uint16_t;  // Servo pulse width in microseconds
    using Error = GpioError;

    Gpio() = default;
    virtual ~Gpio() = default;

    virtual auto set_mode_output(const std::vector<Pin> & pins) -> tl::expected<void, Error> = 0;
    virtual auto set_mode_input(const std::vector<Pin> & pins) -> tl::expected<void, Error> = 0;
    virtual auto write_digital(const Pin & pin, bool && enabled) -> tl::expected<void, Error> = 0;
    virtual auto write_pwm_pulsewidth(const Pin & pin, const PulseWidth && pulsewidth)
        -> tl::expected<void, Error> = 0;
};

}  // namespace sinsei_umiusi_control::hardware_model::interface

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_INTERFACE_GPIO_HPP

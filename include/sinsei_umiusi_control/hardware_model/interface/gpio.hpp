#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_INTERFACE_GPIO_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_INTERFACE_GPIO_HPP

#include <cstdint>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>
#include <vector>

namespace sinsei_umiusi_control::hardware_model::interface {

class Gpio {
  public:
    using Pin = uint32_t;         // GPIO pin number
    using PulseWidth = uint16_t;  // Servo pulse width in microseconds

    Gpio() = default;
    virtual ~Gpio() = default;

    virtual auto set_mode_output(const std::vector<Pin> & pins) -> tl::expected<void, std::string> = 0;
    virtual auto set_mode_input(const std::vector<Pin> & pins) -> tl::expected<void, std::string> = 0;
    virtual auto write_digital(const Pin & pin, bool && enabled) -> tl::expected<void, std::string> = 0;
    virtual auto write_pwm_pulsewidth(const Pin & pin, const PulseWidth && pulsewidth)
        -> tl::expected<void, std::string> = 0;
};

}  // namespace sinsei_umiusi_control::hardware_model::interface

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_INTERFACE_GPIO_HPP

#include "sinsei_umiusi_control/hardware_model/impl/mock_gpio.hpp"

namespace sinsei_umiusi_control::hardware_model::impl {

auto MockGpio::set_mode_output(const std::vector<Pin> & pins) -> tl::expected<void, Error> {
    (void)pins;
    return {};
}

auto MockGpio::set_mode_input(const std::vector<Pin> & pins) -> tl::expected<void, Error> {
    (void)pins;
    return {};
}

auto MockGpio::write_digital(const Pin & pin, bool && enabled) -> tl::expected<void, Error> {
    (void)pin;
    (void)enabled;
    return {};
}

auto MockGpio::write_pwm_pulsewidth(const Pin & pin, const PulseWidth && pulsewidth)
    -> tl::expected<void, Error> {
    (void)pin;
    (void)pulsewidth;
    return {};
}

}  // namespace sinsei_umiusi_control::hardware_model::impl

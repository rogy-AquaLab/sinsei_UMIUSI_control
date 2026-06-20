#include "sinsei_umiusi_control/hardware_model/impl/pigpio.hpp"

#include <pigpio.h>
#include <pigpiod_if2.h>
#include <sys/types.h>

using namespace sinsei_umiusi_control::hardware_model;

impl::Pigpio::Pigpio() { this->pi = ::pigpio_start(NULL, NULL); }

impl::Pigpio::~Pigpio() { ::pigpio_stop(pi); }

auto impl::Pigpio::set_mode_output(const std::vector<Pin> & pins) -> tl::expected<void, Error> {
    for (const auto & pin : pins) {
        const auto res = ::set_mode(this->pi, pin, PI_OUTPUT);
        if (res == 0) {
            continue;
        }
        switch (res) {
            case PI_BAD_GPIO:
                return tl::unexpected(Error::BadGpio);
            case PI_NOT_PERMITTED:
                return tl::unexpected(Error::NotPermitted);
            default:
                return tl::unexpected(Error::UnknownError);
        }
    }
    return {};
}

auto impl::Pigpio::set_mode_input(const std::vector<Pin> & pins) -> tl::expected<void, Error> {
    for (const auto & pin : pins) {
        const auto res = ::set_mode(this->pi, pin, PI_INPUT);
        if (res == 0) {
            continue;
        }
        switch (res) {
            case PI_BAD_GPIO:
                return tl::unexpected(Error::BadGpio);
            case PI_NOT_PERMITTED:
                return tl::unexpected(Error::NotPermitted);
            default:
                return tl::unexpected(Error::UnknownError);
        }
    }
    return {};
}

auto impl::Pigpio::write_digital(const Pin & pin, bool && enabled) -> tl::expected<void, Error> {
    const auto res = ::gpio_write(this->pi, pin, enabled ? 1 : 0);
    if (res == 0) {
        return {};
    }
    switch (res) {
        case PI_BAD_GPIO:
            return tl::unexpected(Error::BadGpio);
        case PI_BAD_LEVEL:
            return tl::unexpected(Error::BadLevel);
        case PI_NOT_PERMITTED:
            return tl::unexpected(Error::NotPermitted);
        default:
            return tl::unexpected(Error::UnknownError);
    }
}

auto impl::Pigpio::write_pwm_pulsewidth(const Pin & pin, const PulseWidth && pulsewidth)
    -> tl::expected<void, Error> {
    const auto res = ::set_servo_pulsewidth(this->pi, pin, pulsewidth);
    if (res == 0) {
        return {};
    }
    switch (res) {
        case PI_BAD_GPIO:
            return tl::unexpected(Error::BadGpio);
        case PI_BAD_PULSEWIDTH:
            return tl::unexpected(Error::BadPulsewidth);
        case PI_NOT_PERMITTED:
            return tl::unexpected(Error::NotPermitted);
        default:
            return tl::unexpected(Error::UnknownError);
    }
}

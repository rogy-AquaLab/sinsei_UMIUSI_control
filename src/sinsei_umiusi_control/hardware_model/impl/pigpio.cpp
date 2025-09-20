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

auto impl::Pigpio::write_pwm_duty(const Pin & pin, const double && duty)
    -> tl::expected<void, Error> {
    const auto res = ::set_PWM_dutycycle(this->pi, pin, static_cast<int>(duty * 255.0));
    if (res == 0) {
        return {};
    }
    switch (res) {
        case PI_BAD_GPIO:
            return tl::unexpected(Error::BadGpio);
        case PI_BAD_DUTYCYCLE:
            return tl::unexpected(Error::BadParameter);
        case PI_NOT_PERMITTED:
            return tl::unexpected(Error::NotPermitted);
        default:
            return tl::unexpected(Error::UnknownError);
    }
}

auto impl::Pigpio::write_pwm_pulsewidth(const Pin & pin, const int && pulsewidth)
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

auto impl::Pigpio::i2c_open(const Addr & address) -> tl::expected<void, Error> {
    this->i2c_address = address;
    const auto res = ::i2c_open(this->pi, I2C_BUS, this->i2c_address, 0U);
    if (res >= 0) {
        this->i2c_handle = res;
        return {};
    }
    switch (res) {
        case PI_BAD_I2C_BUS:
            return tl::unexpected(Error::I2cBadBus);
        case PI_BAD_I2C_ADDR:
            return tl::unexpected(Error::I2CBadAddress);
        case PI_BAD_FLAGS:
            return tl::unexpected(Error::BadFlags);
        case PI_NO_HANDLE:
            return tl::unexpected(Error::NoHandle);
        case PI_I2C_OPEN_FAILED:
            return tl::unexpected(Error::I2cOpenFailed);
        default:
            return tl::unexpected(Error::UnknownError);
    }
}

auto impl::Pigpio::i2c_close() -> tl::expected<void, Error> {
    if (!this->i2c_handle) {
        return tl::unexpected(Error::NoHandle);
    }
    const auto res = ::i2c_close(pi, this->i2c_handle.value());
    switch (res) {
        case 0:
            this->i2c_handle = std::nullopt;
            return {};
        case PI_BAD_HANDLE:
            return tl::unexpected(Error::BadHandle);
        default:
            return tl::unexpected(Error::UnknownError);
    }
}

auto impl::Pigpio::i2c_write_byte(std::byte && value) -> tl::expected<void, Error> {
    if (!this->i2c_handle) {
        return tl::unexpected(Error::NoHandle);
    }
    const auto b_val = std::to_integer<Addr>(value);
    auto res = ::i2c_write_byte(this->pi, this->i2c_handle.value(), b_val);
    switch (res) {
        case 0:
            return {};
        case PI_BAD_HANDLE:
            return tl::unexpected(Error::BadHandle);
        case PI_BAD_PARAM:
            return tl::unexpected(Error::BadParameter);
        case PI_I2C_WRITE_FAILED:
            return tl::unexpected(Error::I2cWriteFailed);
        default:
            return tl::unexpected(Error::UnknownError);
    }
}

auto impl::Pigpio::i2c_read_byte() const -> tl::expected<std::byte, Error> {
    if (!this->i2c_handle) {
        return tl::unexpected(Error::NoHandle);
    }
    const auto res = ::i2c_read_byte(this->pi, this->i2c_handle.value());
    if (res >= 0) {
        return static_cast<std::byte>(res & 0xFF);
    }
    switch (res) {
        case PI_BAD_HANDLE:
            return tl::unexpected(Error::BadHandle);
        case PI_I2C_READ_FAILED:
            return tl::unexpected(Error::I2cReadFailed);
        default:
            return tl::unexpected(Error::UnknownError);
    }
}

auto impl::Pigpio::i2c_write_byte_data(const Addr & reg, std::byte && value)
    -> tl::expected<void, Error> {
    if (!this->i2c_handle) {
        return tl::unexpected(Error::NoHandle);
    }
    const auto b_val = std::to_integer<Addr>(value);
    const auto res = ::i2c_write_byte_data(this->pi, this->i2c_handle.value(), reg, b_val);
    switch (res) {
        case 0:
            return {};
        case PI_BAD_HANDLE:
            return tl::unexpected(Error::BadHandle);
        case PI_BAD_PARAM:
            return tl::unexpected(Error::BadParameter);
        case PI_I2C_WRITE_FAILED:
            return tl::unexpected(Error::I2cWriteFailed);
        default:
            return tl::unexpected(Error::UnknownError);
    }
}

auto impl::Pigpio::i2c_read_byte_data(const Addr & reg) const -> tl::expected<std::byte, Error> {
    if (!this->i2c_handle) {
        return tl::unexpected(Error::NoHandle);
    }
    const auto res = ::i2c_read_byte_data(this->pi, this->i2c_handle.value(), reg);
    if (res >= 0) {
        return static_cast<std::byte>(res & 0xFF);
    }
    switch (res) {
        case PI_BAD_HANDLE:
            return tl::unexpected(Error::BadHandle);
        case PI_BAD_PARAM:
            return tl::unexpected(Error::BadParameter);
        case PI_I2C_READ_FAILED:
            return tl::unexpected(Error::I2cReadFailed);
        default:
            return tl::unexpected(Error::UnknownError);
    }
}

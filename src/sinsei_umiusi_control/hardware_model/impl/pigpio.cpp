#include "sinsei_umiusi_control/hardware_model/impl/pigpio.hpp"

#include <pigpio.h>
#include <pigpiod_if2.h>
#include <sys/types.h>

namespace suchm = sinsei_umiusi_control::hardware_model;

suchm::impl::Pigpio::Pigpio() { this->pi = ::pigpio_start(NULL, NULL); }

suchm::impl::Pigpio::~Pigpio() { ::pigpio_stop(pi); }

auto suchm::impl::Pigpio::set_mode_output(std::vector<GpioPin> pins)
    -> tl::expected<void, GpioError> {
    for (const auto & pin : pins) {
        auto res = ::set_mode(this->pi, pin, PI_OUTPUT);
        if (res == 0) {
            continue;
        }
        switch (res) {
            case PI_BAD_GPIO:
                return tl::unexpected(GpioError::BadGpio);
            case PI_NOT_PERMITTED:
                return tl::unexpected(GpioError::NotPermitted);
            default:
                return tl::unexpected(GpioError::UnknownError);
        }
    }
    return {};
}

auto suchm::impl::Pigpio::set_mode_input(std::vector<GpioPin> pins)
    -> tl::expected<void, GpioError> {
    for (const auto & pin : pins) {
        auto res = ::set_mode(this->pi, pin, PI_INPUT);
        if (res == 0) {
            continue;
        }
        switch (res) {
            case PI_BAD_GPIO:
                return tl::unexpected(GpioError::BadGpio);
            case PI_NOT_PERMITTED:
                return tl::unexpected(GpioError::NotPermitted);
            default:
                return tl::unexpected(GpioError::UnknownError);
        }
    }
    return {};
}

auto suchm::impl::Pigpio::write_digital(GpioPin pin, bool enabled)
    -> tl::expected<void, GpioError> {
    auto res = ::gpio_write(pi, pin, enabled ? 1 : 0);
    if (res == 0) {
        return {};
    }
    switch (res) {
        case PI_BAD_GPIO:
            return tl::unexpected(GpioError::BadGpio);
        case PI_BAD_LEVEL:
            return tl::unexpected(GpioError::BadLevel);
        case PI_NOT_PERMITTED:
            return tl::unexpected(GpioError::NotPermitted);
        default:
            return tl::unexpected(GpioError::UnknownError);
    }
}

auto suchm::impl::Pigpio::write_pwm() -> tl::expected<void, GpioError> {
    // TODO: PWMの処理を実装する
    return tl::unexpected(GpioError::UnknownError);
}

auto suchm::impl::Pigpio::i2c_open(uint32_t address) -> tl::expected<void, GpioError> {
    this->i2c_address = address;
    auto res = ::i2c_open(pi, I2C_BUS, this->i2c_address, 0U);
    if (res >= 0) {
        this->i2c_handle = res;
        return {};
    }
    switch (res) {
        case PI_BAD_I2C_BUS:
            return tl::unexpected(GpioError::I2cBadBus);
        case PI_BAD_I2C_ADDR:
            return tl::unexpected(GpioError::I2CBadAddress);
        case PI_BAD_FLAGS:
            return tl::unexpected(GpioError::BadFlags);
        case PI_NO_HANDLE:
            return tl::unexpected(GpioError::NoHandle);
        case PI_I2C_OPEN_FAILED:
            return tl::unexpected(GpioError::I2cOpenFailed);
        default:
            return tl::unexpected(GpioError::UnknownError);
    }
}

auto suchm::impl::Pigpio::i2c_close() -> tl::expected<void, GpioError> {
    if (!this->i2c_handle) {
        return tl::unexpected(GpioError::NoHandle);
    }
    auto res = ::i2c_close(pi, this->i2c_handle.value());
    switch (res) {
        case 0:
            this->i2c_handle = std::nullopt;
            return {};
        case PI_BAD_HANDLE:
            return tl::unexpected(GpioError::BadHandle);
        default:
            return tl::unexpected(GpioError::UnknownError);
    }
}

auto suchm::impl::Pigpio::i2c_write_byte(std::byte value) -> tl::expected<void, GpioError> {
    if (!this->i2c_handle) {
        return tl::unexpected(GpioError::NoHandle);
    }
    const auto b_val = std::to_integer<uint32_t>(value);
    auto res = ::i2c_write_byte(pi, this->i2c_handle.value(), b_val);
    switch (res) {
        case 0:
            return {};
        case PI_BAD_HANDLE:
            return tl::unexpected(GpioError::BadHandle);
        case PI_BAD_PARAM:
            return tl::unexpected(GpioError::BadParameter);
        case PI_I2C_WRITE_FAILED:
            return tl::unexpected(GpioError::I2cWriteFailed);
        default:
            return tl::unexpected(GpioError::UnknownError);
    }
}

auto suchm::impl::Pigpio::i2c_read_byte() -> tl::expected<std::byte, GpioError> {
    if (!this->i2c_handle) {
        return tl::unexpected(GpioError::NoHandle);
    }
    auto res = ::i2c_read_byte(pi, this->i2c_handle.value());
    if (res >= 0) {
        return static_cast<std::byte>(res & 0xFF);
    }
    switch (res) {
        case PI_BAD_HANDLE:
            return tl::unexpected(GpioError::BadHandle);
        case PI_I2C_READ_FAILED:
            return tl::unexpected(GpioError::I2cReadFailed);
        default:
            return tl::unexpected(GpioError::UnknownError);
    }
}

auto suchm::impl::Pigpio::i2c_write_byte_data(uint32_t reg, std::byte value)
    -> tl::expected<void, GpioError> {
    if (!this->i2c_handle) {
        return tl::unexpected(GpioError::NoHandle);
    }
    const auto b_val = std::to_integer<uint32_t>(value);
    auto res = ::i2c_write_byte_data(pi, this->i2c_handle.value(), reg, b_val);
    switch (res) {
        case 0:
            return {};
        case PI_BAD_HANDLE:
            return tl::unexpected(GpioError::BadHandle);
        case PI_BAD_PARAM:
            return tl::unexpected(GpioError::BadParameter);
        case PI_I2C_WRITE_FAILED:
            return tl::unexpected(GpioError::I2cWriteFailed);
        default:
            return tl::unexpected(GpioError::UnknownError);
    }
}

auto suchm::impl::Pigpio::i2c_read_byte_data(uint32_t reg) -> tl::expected<std::byte, GpioError> {
    if (!this->i2c_handle) {
        return tl::unexpected(GpioError::NoHandle);
    }
    auto res = ::i2c_read_byte_data(pi, this->i2c_handle.value(), reg);
    if (res >= 0) {
        return static_cast<std::byte>(res & 0xFF);
    }
    switch (res) {
        case PI_BAD_HANDLE:
            return tl::unexpected(GpioError::BadHandle);
        case PI_BAD_PARAM:
            return tl::unexpected(GpioError::BadParameter);
        case PI_I2C_READ_FAILED:
            return tl::unexpected(GpioError::I2cReadFailed);
        default:
            return tl::unexpected(GpioError::UnknownError);
    }
}
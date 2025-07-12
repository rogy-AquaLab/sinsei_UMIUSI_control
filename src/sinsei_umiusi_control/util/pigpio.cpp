#include "sinsei_umiusi_control/util/pigpio.hpp"

#include <pigpio.h>
#include <pigpiod_if2.h>

namespace suc_util = sinsei_umiusi_control::util;

suc_util::Pigpio::Pigpio() { this->pi = pigpio_start(NULL, NULL); }

suc_util::Pigpio::~Pigpio() { pigpio_stop(pi); }

auto suc_util::Pigpio::set_mode_output(std::vector<uint8_t> pins) -> tl::expected<void, GpioError> {
    for (const auto & pin : pins) {
        auto res = set_mode(this->pi, pin, PI_OUTPUT);
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

auto suc_util::Pigpio::set_mode_input(std::vector<uint8_t> pins) -> tl::expected<void, GpioError> {
    for (const auto & pin : pins) {
        auto res = set_mode(this->pi, pin, PI_INPUT);
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

auto suc_util::Pigpio::write_digital(uint8_t pin, bool enabled) -> tl::expected<void, GpioError> {
    auto res = gpio_write(pi, pin, enabled ? 1 : 0);
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

auto suc_util::Pigpio::write_pwm() -> tl::expected<void, GpioError> {
    // TODO: PWMの処理を実装する
    return tl::unexpected(GpioError::UnknownError);
}

auto suc_util::Pigpio::i2c_open(int address) -> tl::expected<void, GpioError> {
    this->i2c_address = address;
    auto res = ::i2c_open(pi, I2C_BUS, this->i2c_address, 0);
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

auto suc_util::Pigpio::i2c_close() -> tl::expected<void, GpioError> {
    if (this->i2c_handle < 0) {
        return tl::unexpected(GpioError::NoHandle);
    }
    auto res = ::i2c_close(pi, this->i2c_handle);
    switch (res) {
        case 0:
            this->i2c_handle = -1;
            return {};
        case PI_BAD_HANDLE:
            return tl::unexpected(GpioError::BadHandle);
        default:
            return tl::unexpected(GpioError::UnknownError);
    }
}

auto suc_util::Pigpio::i2c_write_byte(uint8_t value) -> tl::expected<void, GpioError> {
    if (this->i2c_handle < 0) {
        return tl::unexpected(GpioError::NoHandle);
    }
    auto res = ::i2c_write_byte(pi, this->i2c_handle, value);
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

auto suc_util::Pigpio::i2c_read_byte() -> tl::expected<uint8_t, GpioError> {
    if (this->i2c_handle < 0) {
        return tl::unexpected(GpioError::NoHandle);
    }
    auto res = ::i2c_read_byte(pi, this->i2c_handle);
    if (res >= 0) {
        return static_cast<uint8_t>(res);
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

auto suc_util::Pigpio::i2c_write_byte_data(uint8_t reg, uint8_t value)
    -> tl::expected<void, GpioError> {
    if (this->i2c_handle < 0) {
        return tl::unexpected(GpioError::NoHandle);
    }
    auto res = ::i2c_write_byte_data(pi, this->i2c_handle, reg, value);
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

auto suc_util::Pigpio::i2c_read_byte_data(uint8_t reg) -> tl::expected<uint8_t, GpioError> {
    if (this->i2c_handle < 0) {
        return tl::unexpected(GpioError::NoHandle);
    }
    auto res = ::i2c_read_byte_data(pi, this->i2c_handle, reg);
    if (res >= 0) {
        return static_cast<uint8_t>(res);
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
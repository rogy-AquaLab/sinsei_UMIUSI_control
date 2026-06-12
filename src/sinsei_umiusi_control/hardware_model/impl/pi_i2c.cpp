#include "sinsei_umiusi_control/hardware_model/impl/pi_i2c.hpp"

#include <pigpio.h>
#include <pigpiod_if2.h>
#include <sys/types.h>

using namespace sinsei_umiusi_control::hardware_model;

impl::PiI2c::PiI2c() { this->pi = ::pigpio_start(NULL, NULL); }

impl::PiI2c::~PiI2c() {
    if (this->i2c_handle) {
        (void)this->close();
    }
    ::pigpio_stop(this->pi);
}

auto impl::PiI2c::open(const Addr & address) -> tl::expected<void, Error> {
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

auto impl::PiI2c::close() -> tl::expected<void, Error> {
    if (!this->i2c_handle) {
        return tl::unexpected(Error::I2cNotOpen);
    }
    const auto res = ::i2c_close(this->pi, this->i2c_handle.value());
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

auto impl::PiI2c::write_byte(std::byte && value) -> tl::expected<void, Error> {
    if (!this->i2c_handle) {
        return tl::unexpected(Error::I2cNotOpen);
    }
    const auto byte_value = std::to_integer<Addr>(value);
    const auto res = ::i2c_write_byte(this->pi, this->i2c_handle.value(), byte_value);
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

auto impl::PiI2c::read_byte() const -> tl::expected<std::byte, Error> {
    if (!this->i2c_handle) {
        return tl::unexpected(Error::I2cNotOpen);
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

auto impl::PiI2c::write_byte_data(const Addr & reg, std::byte && value)
    -> tl::expected<void, Error> {
    if (!this->i2c_handle) {
        return tl::unexpected(Error::I2cNotOpen);
    }
    const auto byte_value = std::to_integer<Addr>(value);
    const auto res = ::i2c_write_byte_data(this->pi, this->i2c_handle.value(), reg, byte_value);
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

auto impl::PiI2c::read_byte_data(const Addr & reg) const -> tl::expected<std::byte, Error> {
    if (!this->i2c_handle) {
        return tl::unexpected(Error::I2cNotOpen);
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

auto impl::PiI2c::read_block_data(const Addr & reg, std::byte * buffer, const size_t length) const
    -> tl::expected<void, Error> {
    if (!this->i2c_handle) {
        return tl::unexpected(Error::I2cNotOpen);
    }
    const auto res = ::i2c_read_i2c_block_data(
        this->pi, this->i2c_handle.value(), reg, reinterpret_cast<char *>(buffer), length);

    if (res == static_cast<int>(length)) {
        return {};
    }
    if (res >= 0) {
        return tl::unexpected(Error::I2cReadFailed);
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

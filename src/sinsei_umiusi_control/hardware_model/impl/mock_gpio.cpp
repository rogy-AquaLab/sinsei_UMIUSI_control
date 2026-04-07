#include "sinsei_umiusi_control/hardware_model/impl/mock_gpio.hpp"

#include <cstring>

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

auto MockGpio::i2c_open(const Addr & address) -> tl::expected<void, Error> {
    this->i2c_opened_ = true;
    this->current_i2c_address_ = address;
    return {};
}

auto MockGpio::i2c_close() -> tl::expected<void, Error> {
    this->i2c_opened_ = false;
    this->current_i2c_address_ = 0;
    return {};
}

auto MockGpio::i2c_write_byte(std::byte && value) -> tl::expected<void, Error> {
    (void)value;
    if (!this->i2c_opened_) {
        return tl::unexpected(Error::I2cNotOpen);
    }
    return {};
}

auto MockGpio::i2c_read_byte() const -> tl::expected<std::byte, Error> {
    if (!this->i2c_opened_) {
        return tl::unexpected(Error::I2cNotOpen);
    }
    return std::byte{0x00};
}

auto MockGpio::i2c_write_byte_data(const Addr & reg, std::byte && value)
    -> tl::expected<void, Error> {
    (void)reg;
    (void)value;
    if (!this->i2c_opened_) {
        return tl::unexpected(Error::I2cNotOpen);
    }
    return {};
}

auto MockGpio::i2c_read_byte_data(const Addr & reg) const -> tl::expected<std::byte, Error> {
    (void)reg;
    if (!this->i2c_opened_) {
        return tl::unexpected(Error::I2cNotOpen);
    }
    return std::byte{0x00};
}

auto MockGpio::i2c_read_block_data(const Addr & reg, std::byte * buffer, const size_t length) const
    -> tl::expected<void, Error> {
    (void)reg;
    if (!this->i2c_opened_) {
        return tl::unexpected(Error::I2cNotOpen);
    }
    if (buffer != nullptr && length > 0) {
        std::memset(buffer, 0x00, length);
    }
    return {};
}

}  // namespace sinsei_umiusi_control::hardware_model::impl

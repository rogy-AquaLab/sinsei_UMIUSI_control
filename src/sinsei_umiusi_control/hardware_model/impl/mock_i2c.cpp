#include "sinsei_umiusi_control/hardware_model/impl/mock_i2c.hpp"

#include <cstring>

namespace sinsei_umiusi_control::hardware_model::impl {

auto MockI2c::open(const Addr & address) -> tl::expected<void, Error> {
    this->opened_ = true;
    this->current_address_ = address;
    return {};
}

auto MockI2c::close() -> tl::expected<void, Error> {
    this->opened_ = false;
    this->current_address_ = 0;
    return {};
}

auto MockI2c::write_byte(std::byte && value) -> tl::expected<void, Error> {
    (void)value;
    if (!this->opened_) {
        return tl::unexpected(Error::NotOpen);
    }
    return {};
}

auto MockI2c::read_byte() const -> tl::expected<std::byte, Error> {
    if (!this->opened_) {
        return tl::unexpected(Error::NotOpen);
    }
    return std::byte{0x00};
}

auto MockI2c::write_byte_data(const Addr & reg, std::byte && value) -> tl::expected<void, Error> {
    (void)reg;
    (void)value;
    if (!this->opened_) {
        return tl::unexpected(Error::NotOpen);
    }
    return {};
}

auto MockI2c::read_byte_data(const Addr & reg) const -> tl::expected<std::byte, Error> {
    (void)reg;
    if (!this->opened_) {
        return tl::unexpected(Error::NotOpen);
    }
    return std::byte{0x00};
}

auto MockI2c::read_block_data(const Addr & reg, std::byte * buffer, const size_t length) const
    -> tl::expected<void, Error> {
    (void)reg;
    if (!this->opened_) {
        return tl::unexpected(Error::NotOpen);
    }
    if (buffer != nullptr && length > 0) {
        std::memset(buffer, 0x00, length);
    }
    return {};
}

}  // namespace sinsei_umiusi_control::hardware_model::impl

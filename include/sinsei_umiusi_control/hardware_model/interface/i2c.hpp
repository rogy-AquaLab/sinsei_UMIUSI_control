#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_INTERFACE_I2C_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_INTERFACE_I2C_HPP

#include <cstddef>
#include <cstdint>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>

namespace sinsei_umiusi_control::hardware_model::interface {

enum class I2cError {
    BadFlags,
    BadHandle,
    BadParameter,
    NoHandle,
    I2cNotOpen,
    I2cBadBus,
    I2CBadAddress,
    I2cOpenFailed,
    I2cWriteFailed,
    I2cReadFailed,
    UnknownError,
};

inline auto i2c_error_to_string(const I2cError & error) -> std::string {
    switch (error) {
        case I2cError::BadFlags:
            return "Bad flags specified";
        case I2cError::BadHandle:
            return "Bad handle specified";
        case I2cError::BadParameter:
            return "Bad parameter specified";
        case I2cError::NoHandle:
            return "No handle available";
        case I2cError::I2cNotOpen:
            return "I2C not open";
        case I2cError::I2cBadBus:
            return "I2C bad bus";
        case I2cError::I2CBadAddress:
            return "I2C bad address";
        case I2cError::I2cOpenFailed:
            return "I2C open failed";
        case I2cError::I2cWriteFailed:
            return "I2C write failed";
        case I2cError::I2cReadFailed:
            return "I2C read failed";
        default:
            return "Unknown error";
    }
}

class I2c {
  public:
    using Addr = uint32_t;
    using Error = I2cError;

    I2c() = default;
    virtual ~I2c() = default;

    virtual auto open(const Addr & address) -> tl::expected<void, Error> = 0;
    virtual auto close() -> tl::expected<void, Error> = 0;
    virtual auto write_byte(std::byte && value) -> tl::expected<void, Error> = 0;
    virtual auto read_byte() const -> tl::expected<std::byte, Error> = 0;
    virtual auto write_byte_data(const Addr & reg, std::byte && value)
        -> tl::expected<void, Error> = 0;
    virtual auto read_byte_data(const Addr & reg) const -> tl::expected<std::byte, Error> = 0;
    virtual auto read_block_data(const Addr & reg, std::byte * buffer, const size_t length) const
        -> tl::expected<void, Error> = 0;
};

}  // namespace sinsei_umiusi_control::hardware_model::interface

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_INTERFACE_I2C_HPP

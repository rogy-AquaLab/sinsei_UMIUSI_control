#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_INTERFACE_I2C_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_INTERFACE_I2C_HPP

#include <cstddef>
#include <cstdint>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>

namespace sinsei_umiusi_control::hardware_model::interface {

enum class I2cError {
    NotOpen,
    BadBus,
    BadAddress,
    OpenFailed,
    WriteFailed,
    ReadFailed,
    UnknownError,
};

enum class I2cDirection { Read, Write };

struct I2cMessage {
    I2cDirection direction;
    std::byte * data;
    size_t length;
};

inline auto i2c_error_to_string(const I2cError & error) -> std::string {
    switch (error) {
        case I2cError::NotOpen:
            return "I2C not open";
        case I2cError::BadBus:
            return "I2C bad bus";
        case I2cError::BadAddress:
            return "I2C bad address";
        case I2cError::OpenFailed:
            return "I2C open failed";
        case I2cError::WriteFailed:
            return "I2C write failed";
        case I2cError::ReadFailed:
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
    virtual auto transfer(const std::vector<I2cMessage> & msgs) -> tl::expected<void, Error> = 0;
};

}  // namespace sinsei_umiusi_control::hardware_model::interface

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_INTERFACE_I2C_HPP

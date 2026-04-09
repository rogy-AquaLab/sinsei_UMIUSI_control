#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_INTERFACE_I2C_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_INTERFACE_I2C_HPP

#include <cstddef>
#include <cstdint>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>

namespace sinsei_umiusi_control::hardware_model::interface {

// 意味が異なるのに型が似ている概念が多いためNewTypeを定義して区別する
struct I2cDeviceAddr {
    uint16_t value;
};
struct I2cRegisterAddr {
    std::byte value;
};

enum class I2cDirection { Read, Write };

struct I2cBufferView {
    std::byte * data;
    size_t length;
};

struct I2cMessage {
    using Flags = uint16_t;

    I2cDeviceAddr address;
    I2cDirection direction;
    I2cBufferView buffer;
    Flags flags;
};

class I2c {
  public:
    I2c() = default;
    virtual ~I2c() = default;

    virtual auto open() -> tl::expected<void, std::string> = 0;
    virtual auto close() -> tl::expected<void, std::string> = 0;

    virtual auto transfer(const I2cMessage * msgs, std::size_t size)
        -> tl::expected<void, std::string> = 0;

    auto write_reg(I2cDeviceAddr addr, I2cRegisterAddr reg, std::byte value)
        -> tl::expected<void, std::string> {
        std::byte data[] = {reg.value, value};
        I2cMessage msg{addr, I2cDirection::Write, {data, 2}, 0};
        return transfer(&msg, 1);
    }

    auto read_reg(I2cDeviceAddr addr, I2cRegisterAddr reg, I2cBufferView buffer)
        -> tl::expected<void, std::string> {
        std::byte reg_addr[] = {reg.value};
        I2cMessage msgs[2] = {
            {addr, I2cDirection::Write, {reinterpret_cast<std::byte *>(reg_addr), 1}, 0},
            {addr, I2cDirection::Read, buffer, 0}};
        return transfer(msgs, 2);
    }
};

}  // namespace sinsei_umiusi_control::hardware_model::interface

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_INTERFACE_I2C_HPP

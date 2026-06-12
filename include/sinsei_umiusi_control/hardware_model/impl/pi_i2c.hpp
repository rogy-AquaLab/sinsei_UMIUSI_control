#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMPL_PI_I2C_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMPL_PI_I2C_HPP

#include <cstddef>
#include <optional>
#include <rcpputils/tl_expected/expected.hpp>

#include "sinsei_umiusi_control/hardware_model/interface/i2c.hpp"

namespace sinsei_umiusi_control::hardware_model::impl {

class PiI2c : public interface::I2c {
  public:
    using Addr = interface::I2c::Addr;
    using Error = interface::I2c::Error;

    static constexpr uint32_t I2C_BUS = 1;

  private:
    int pi;

    std::optional<uint32_t> i2c_handle = std::nullopt;
    uint32_t i2c_address{};

  public:
    PiI2c();
    ~PiI2c() override;

    auto open(const Addr & address) -> tl::expected<void, Error> override;
    auto close() -> tl::expected<void, Error> override;
    auto write_byte(std::byte && value) -> tl::expected<void, Error> override;
    auto read_byte() const -> tl::expected<std::byte, Error> override;
    auto write_byte_data(const Addr & reg, std::byte && value) -> tl::expected<void, Error>
        override;
    auto read_byte_data(const Addr & reg) const -> tl::expected<std::byte, Error> override;
    auto read_block_data(const Addr & reg, std::byte * buffer, const size_t length) const
        -> tl::expected<void, Error> override;
};

}  // namespace sinsei_umiusi_control::hardware_model::impl

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMPL_PI_I2C_HPP

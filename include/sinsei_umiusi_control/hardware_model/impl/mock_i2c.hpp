#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMPL_MOCK_I2C_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMPL_MOCK_I2C_HPP

#include "sinsei_umiusi_control/hardware_model/interface/i2c.hpp"

namespace sinsei_umiusi_control::hardware_model::impl {

class MockI2c : public interface::I2c {
  public:
    auto open(const Addr & address) -> tl::expected<void, Error> override;
    auto close() -> tl::expected<void, Error> override;
    auto write_byte(std::byte && value) -> tl::expected<void, Error> override;
    auto read_byte() const -> tl::expected<std::byte, Error> override;
    auto write_byte_data(const Addr & reg, std::byte && value) -> tl::expected<void, Error> override;
    auto read_byte_data(const Addr & reg) const -> tl::expected<std::byte, Error> override;
    auto read_block_data(const Addr & reg, std::byte * buffer, const size_t length) const
        -> tl::expected<void, Error> override;

  private:
    bool opened_ = false;
    Addr current_address_ = 0;
};

}  // namespace sinsei_umiusi_control::hardware_model::impl

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMPL_MOCK_I2C_HPP

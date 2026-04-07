#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMPL_MOCK_GPIO_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMPL_MOCK_GPIO_HPP

#include <vector>

#include "sinsei_umiusi_control/hardware_model/interface/gpio.hpp"

namespace sinsei_umiusi_control::hardware_model::impl {

class MockGpio : public interface::Gpio {
  public:
    auto set_mode_output(const std::vector<Pin> & pins) -> tl::expected<void, Error> override;
    auto set_mode_input(const std::vector<Pin> & pins) -> tl::expected<void, Error> override;
    auto write_digital(const Pin & pin, bool && enabled) -> tl::expected<void, Error> override;
    auto write_pwm_pulsewidth(const Pin & pin, const PulseWidth && pulsewidth)
        -> tl::expected<void, Error> override;

    auto i2c_open(const Addr & address) -> tl::expected<void, Error> override;
    auto i2c_close() -> tl::expected<void, Error> override;
    auto i2c_write_byte(std::byte && value) -> tl::expected<void, Error> override;
    auto i2c_read_byte() const -> tl::expected<std::byte, Error> override;
    auto i2c_write_byte_data(const Addr & reg, std::byte && value)
        -> tl::expected<void, Error> override;
    auto i2c_read_byte_data(const Addr & reg) const -> tl::expected<std::byte, Error> override;
    auto i2c_read_block_data(const Addr & reg, std::byte * buffer, const size_t length) const
        -> tl::expected<void, Error> override;

  private:
    bool i2c_opened_ = false;
    Addr current_i2c_address_ = 0;
};

}  // namespace sinsei_umiusi_control::hardware_model::impl

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMPL_MOCK_GPIO_HPP

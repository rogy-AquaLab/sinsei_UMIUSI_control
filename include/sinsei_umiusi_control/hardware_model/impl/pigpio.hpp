#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMPL_PIGPIO_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMPL_PIGPIO_HPP

#include <cstddef>
#include <optional>
#include <rcpputils/tl_expected/expected.hpp>
#include <vector>

#include "sinsei_umiusi_control/hardware_model/interface/gpio.hpp"

namespace sinsei_umiusi_control::hardware_model::impl {

class Pigpio : public interface::Gpio {
  public:
    using Pin = interface::Gpio::Pin;
    using Addr = interface::Gpio::Addr;
    using Error = interface::Gpio::Error;

    static constexpr uint32_t I2C_BUS = 1;

  private:
    int pi;

    std::optional<uint32_t> i2c_handle = std::nullopt;
    uint32_t i2c_address;

  public:
    // 失敗しうるので注意
    Pigpio();
    ~Pigpio();

    auto set_mode_output(const std::vector<Pin> & pins) -> tl::expected<void, Error> override;
    auto set_mode_input(const std::vector<Pin> & pins) -> tl::expected<void, Error> override;
    auto write_digital(const Pin & pin, bool && enabled) -> tl::expected<void, Error> override;
    auto write_pwm_duty(const Pin & pin, const double && duty)
        -> tl::expected<void, Error> override;
    auto write_servo_pulsewidth(const Pin & pin, const uint16_t && pulsewidth)
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
};

}  // namespace sinsei_umiusi_control::hardware_model::impl

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMPL_PIGPIO_HPP

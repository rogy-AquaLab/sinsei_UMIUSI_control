#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMPL_PIGPIO_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMPL_PIGPIO_HPP

#include <cstddef>
#include <optional>
#include <rcpputils/tl_expected/expected.hpp>
#include <vector>

#include "sinsei_umiusi_control/hardware_model/interface/gpio.hpp"

namespace sinsei_umiusi_control::hardware_model::impl {

class Pigpio : public interface::Gpio {
  private:
    using GpioPin = interface::GpioPin;
    using GpioError = interface::GpioError;

    int pi;

    static constexpr uint32_t I2C_BUS = 1;
    std::optional<uint32_t> i2c_handle = std::nullopt;
    uint32_t i2c_address;

  public:
    // 失敗しうるので注意
    Pigpio();
    ~Pigpio();

    auto set_mode_output(std::vector<GpioPin> pins) -> tl::expected<void, GpioError> override;
    auto set_mode_input(std::vector<GpioPin> pins) -> tl::expected<void, GpioError> override;
    auto write_digital(GpioPin pin, bool enabled) -> tl::expected<void, GpioError> override;
    auto write_pwm() -> tl::expected<void, GpioError> override;

    auto i2c_open(uint32_t address) -> tl::expected<void, GpioError> override;
    auto i2c_close() -> tl::expected<void, GpioError> override;
    auto i2c_write_byte(std::byte value) -> tl::expected<void, GpioError> override;
    auto i2c_read_byte() -> tl::expected<std::byte, GpioError> override;
    auto i2c_write_byte_data(uint32_t reg, std::byte value)
        -> tl::expected<void, GpioError> override;
    auto i2c_read_byte_data(uint32_t reg) -> tl::expected<std::byte, GpioError> override;
};

}  // namespace sinsei_umiusi_control::hardware_model::impl

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMPL_PIGPIO_HPP
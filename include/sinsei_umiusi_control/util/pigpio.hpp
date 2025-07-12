#ifndef SINSEI_UMIUSI_CONTROL_UTIL_PIGPIO_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_PIGPIO_HPP

#include <rcpputils/tl_expected/expected.hpp>
#include <vector>

#include "sinsei_umiusi_control/util/gpio_interface.hpp"

namespace sinsei_umiusi_control::util {

class Pigpio : public GpioInterface {
  private:
    int pi;

    static constexpr int I2C_BUS = 1;
    int i2c_handle = -1;
    int i2c_address;

  public:
    // 失敗しうるので注意
    Pigpio();
    ~Pigpio();

    auto set_mode_output(std::vector<uint8_t> pins) -> tl::expected<void, GpioError> override;
    auto set_mode_input(std::vector<uint8_t> pins) -> tl::expected<void, GpioError> override;
    auto write_digital(uint8_t pin, bool enabled) -> tl::expected<void, GpioError> override;
    auto write_pwm() -> tl::expected<void, GpioError> override;

    auto i2c_open(int address) -> tl::expected<void, GpioError> override;
    auto i2c_close() -> tl::expected<void, GpioError> override;
    auto i2c_write_byte(uint8_t value) -> tl::expected<void, GpioError> override;
    auto i2c_read_byte() -> tl::expected<uint8_t, GpioError> override;
    auto i2c_write_byte_data(uint8_t reg, uint8_t value) -> tl::expected<void, GpioError> override;
    auto i2c_read_byte_data(uint8_t reg) -> tl::expected<uint8_t, GpioError> override;
};

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_PIGPIO_HPP
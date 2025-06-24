#ifndef SINSEI_UMIUSI_CONTROL_UTIL_GPIO_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_GPIO_HPP

namespace sinsei_umiusi_control::util {

enum class GpioResult {
    Success,
    Error,
};

class Gpio {
  public:
    Gpio() = default;
    virtual ~Gpio() = default;
    virtual auto write_digital(bool enabled) -> GpioResult = 0;
    virtual auto write_pwm() -> GpioResult = 0;
};

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_GPIO_HPP
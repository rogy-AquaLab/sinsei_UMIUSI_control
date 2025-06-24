#ifndef SINSEI_UMIUSI_CONTROL_UTIL_PIGPIO_HPP
#define SINSEI_UMIUSI_CONTROL_UTIL_PIGPIO_HPP

#include <pigpiod_if2.h>

#include "sinsei_umiusi_control/util/gpio.hpp"

namespace sinsei_umiusi_control::util {

class Pigpio : public Gpio {
  private:
    int pi;

    int pin_number;

  public:
    Pigpio(int pin_number);
    ~Pigpio();

    auto write_digital(bool enabled) -> GpioResult override;
    auto write_pwm() -> GpioResult override;
};

}  // namespace sinsei_umiusi_control::util

#endif  // SINSEI_UMIUSI_CONTROL_UTIL_PIGPIO_HPP
#include "sinsei_umiusi_control/util/pigpio.hpp"

namespace suc_util = sinsei_umiusi_control::util;

suc_util::Pigpio::Pigpio(int pin_number) {
    this->pin_number = pin_number;
    this->pi = pigpio_start(NULL, NULL);
    set_mode(pi, pin_number, PI_OUTPUT);
}

suc_util::Pigpio::~Pigpio() { pigpio_stop(pi); }

auto suc_util::Pigpio::write_digital(bool enabled) -> GpioResult {
    gpio_write(pi, this->pin_number, enabled ? 1 : 0);
    return GpioResult::Success;
}

auto suc_util::Pigpio::write_pwm() -> GpioResult {
    // TODO: PWMの処理を実装する
    return GpioResult::Error;
}
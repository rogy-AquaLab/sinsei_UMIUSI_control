#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMPL_PIGPIO_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMPL_PIGPIO_HPP

#include <rcpputils/tl_expected/expected.hpp>
#include <vector>

#include "sinsei_umiusi_control/hardware_model/interface/gpio.hpp"

namespace sinsei_umiusi_control::hardware_model::impl {

class Pigpio : public interface::Gpio {
  public:
    using Pin = interface::Gpio::Pin;
    using Error = interface::Gpio::Error;

  private:
    int pi;

  public:
    // 失敗しうるので注意
    Pigpio();
    ~Pigpio();

    auto set_mode_output(const std::vector<Pin> & pins) -> tl::expected<void, Error> override;
    auto set_mode_input(const std::vector<Pin> & pins) -> tl::expected<void, Error> override;
    auto write_digital(const Pin & pin, bool && enabled) -> tl::expected<void, Error> override;
    auto write_pwm_pulsewidth(const Pin & pin, const PulseWidth && pulsewidth)
        -> tl::expected<void, Error> override;
};

}  // namespace sinsei_umiusi_control::hardware_model::impl

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMPL_PIGPIO_HPP

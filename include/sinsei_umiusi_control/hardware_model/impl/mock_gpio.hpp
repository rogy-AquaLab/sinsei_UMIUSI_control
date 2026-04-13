#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMPL_MOCK_GPIO_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMPL_MOCK_GPIO_HPP

#include <vector>

#include "sinsei_umiusi_control/hardware_model/interface/gpio.hpp"

namespace sinsei_umiusi_control::hardware_model::impl {

class MockGpio : public interface::Gpio {
  public:
    auto set_mode_output(const std::vector<Pin> & pins) -> tl::expected<void, std::string> override;
    auto set_mode_input(const std::vector<Pin> & pins) -> tl::expected<void, std::string> override;
    auto write_digital(const Pin & pin, bool && enabled) -> tl::expected<void, std::string> override;
    auto write_pwm_pulsewidth(const Pin & pin, const PulseWidth && pulsewidth)
        -> tl::expected<void, std::string> override;
};

}  // namespace sinsei_umiusi_control::hardware_model::impl

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_MODEL_IMPL_MOCK_GPIO_HPP

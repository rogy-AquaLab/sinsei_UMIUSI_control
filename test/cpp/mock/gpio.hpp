#ifndef SINSEI_UMIUSI_CONTROL_test_cpp_mock_GPIO_HPP
#define SINSEI_UMIUSI_CONTROL_test_cpp_mock_GPIO_HPP

#include <gmock/gmock.h>
#include "sinsei_umiusi_control/hardware_model/interface/gpio.hpp"

namespace sinsei_umiusi_control::test::mock {

class Gpio : public hardware_model::interface::Gpio {
  public:
    MOCK_METHOD1(
        set_mode_output, tl::expected<void, std::string>(
                             const std::vector<hardware_model::interface::Gpio::Pin> & pins));
    MOCK_METHOD1(
        set_mode_input, tl::expected<void, std::string>(
                            const std::vector<hardware_model::interface::Gpio::Pin> & pins));
    MOCK_METHOD2(
        write_digital, tl::expected<void, std::string>(
                           const hardware_model::interface::Gpio::Pin & pin, bool && enabled));
    MOCK_METHOD2(
        write_pwm_pulsewidth, tl::expected<void, std::string>(
                                  const hardware_model::interface::Gpio::Pin & pin,
                                  const hardware_model::interface::Gpio::PulseWidth && pulsewidth));
};
}  // namespace sinsei_umiusi_control::test::mock

#endif  // SINSEI_UMIUSI_CONTROL_test_cpp_mock_GPIO_HPP

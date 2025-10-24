#ifndef SINSEI_UMIUSI_CONTROL_hardware_model_THRUSTER_DIRECT_ESC_DIRECT_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_hardware_model_THRUSTER_DIRECT_ESC_DIRECT_MODEL_HPP

#include <memory>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>

#include "sinsei_umiusi_control/cmd/thruster/esc.hpp"
#include "sinsei_umiusi_control/hardware_model/interface/gpio.hpp"

namespace sinsei_umiusi_control::hardware_model::thruster_direct {

class EscDirectModel {
  private:
    std::unique_ptr<interface::Gpio> gpio;
    interface::Gpio::Pin esc_pin;

    interface::Gpio::PulseWidth center_pulse_width;
    interface::Gpio::PulseWidth negative_pulse_width_radius;
    interface::Gpio::PulseWidth positive_pulse_width_radius;

  public:
    EscDirectModel(
        std::unique_ptr<interface::Gpio> gpio, interface::Gpio::Pin esc_pin,
        interface::Gpio::PulseWidth center_pulse_width,
        interface::Gpio::PulseWidth negative_pulse_width_radius,
        interface::Gpio::PulseWidth positive_pulse_width_radius);
    auto on_init() -> tl::expected<void, std::string>;
    auto on_read() const -> tl::expected<void, std::string>;
    auto on_write(
        sinsei_umiusi_control::cmd::thruster::esc::Enabled && enabled,
        sinsei_umiusi_control::cmd::thruster::esc::DutyCycle && duty_cycle)
        -> tl::expected<void, std::string>;
};

}  // namespace sinsei_umiusi_control::hardware_model::thruster_direct

#endif  // SINSEI_UMIUSI_CONTROL_hardware_model_THRUSTER_DIRECT_ESC_DIRECT_MODEL_HPP

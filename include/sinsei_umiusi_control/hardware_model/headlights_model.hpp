#ifndef SINSEI_UMIUSI_CONTROL_hardware_model_HEADLIGHTS_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_hardware_model_HEADLIGHTS_MODEL_HPP

#include <memory>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>

#include "sinsei_umiusi_control/cmd/headlights.hpp"
#include "sinsei_umiusi_control/hardware_model/interface/gpio_interface.hpp"

namespace sinsei_umiusi_control::hardware_model {

class HeadlightsModel {
  private:
    std::unique_ptr<interface::GpioInterface> gpio;
    interface::GpioPin high_beam_pin;
    interface::GpioPin low_beam_pin;
    interface::GpioPin ir_pin;

  public:
    HeadlightsModel(
        std::unique_ptr<interface::GpioInterface> gpio, interface::GpioPin high_beam_pin,
        interface::GpioPin low_beam_pin, interface::GpioPin ir_pin);
    auto on_init() -> tl::expected<void, std::string>;
    auto on_read() -> tl::expected<void, std::string>;
    auto on_write(
        sinsei_umiusi_control::cmd::headlights::HighBeamEnabled & high_beam_enabled,
        sinsei_umiusi_control::cmd::headlights::LowBeamEnabled & low_beam_enabled,
        sinsei_umiusi_control::cmd::headlights::IrEnabled & ir_enabled)
        -> tl::expected<void, std::string>;
};

}  // namespace sinsei_umiusi_control::hardware_model

#endif  // SINSEI_UMIUSI_CONTROL_hardware_model_HEADLIGHTS_MODEL_HPP
#ifndef SINSEI_UMIUSI_CONTROL_hardware_model_HEADLIGHTS_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_hardware_model_HEADLIGHTS_MODEL_HPP

#include <memory>

#include "sinsei_umiusi_control/cmd/headlights.hpp"
#include "sinsei_umiusi_control/util/gpio_interface.hpp"

namespace suc = sinsei_umiusi_control;

namespace sinsei_umiusi_control::hardware_model {

class HeadlightsModel {
  private:
    std::unique_ptr<util::GpioInterface> high_beam_gpio;
    std::unique_ptr<util::GpioInterface> low_beam_gpio;
    std::unique_ptr<util::GpioInterface> ir_gpio;

  public:
    HeadlightsModel(
        std::unique_ptr<util::GpioInterface> high_beam_gpio,
        std::unique_ptr<util::GpioInterface> low_beam_gpio,
        std::unique_ptr<util::GpioInterface> ir_gpio);
    auto on_read() -> void;
    auto on_write(
        sinsei_umiusi_control::cmd::headlights::HighBeamEnabled & high_beam_enabled,
        sinsei_umiusi_control::cmd::headlights::LowBeamEnabled & low_beam_enabled,
        sinsei_umiusi_control::cmd::headlights::IrEnabled & ir_enabled) -> void;
};

}  // namespace sinsei_umiusi_control::hardware_model

#endif  // SINSEI_UMIUSI_CONTROL_hardware_model_HEADLIGHTS_MODEL_HPP
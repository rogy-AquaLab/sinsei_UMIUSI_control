#ifndef SINSEI_UMIUSI_CONTROL_hardware_model_HEADLIGHTS_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_hardware_model_HEADLIGHTS_MODEL_HPP

#include <cstdint>
#include <memory>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>

#include "sinsei_umiusi_control/cmd/headlights.hpp"
#include "sinsei_umiusi_control/util/gpio_interface.hpp"

namespace sinsei_umiusi_control::hardware_model {

class HeadlightsModel {
  private:
    std::unique_ptr<util::GpioInterface> gpio;
    uint8_t high_beam_pin;
    uint8_t low_beam_pin;
    uint8_t ir_pin;

  public:
    HeadlightsModel(
        std::unique_ptr<util::GpioInterface> gpio, uint8_t high_beam_pin, uint8_t low_beam_pin,
        uint8_t ir_pin);
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
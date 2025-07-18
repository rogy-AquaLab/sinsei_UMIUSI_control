#ifndef SINSEI_UMIUSI_CONTROL_hardware_model_INDICATOR_LED_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_hardware_model_INDICATOR_LED_MODEL_HPP

#include <memory>
#include <rcpputils/tl_expected/expected.hpp>
#include <string>

#include "sinsei_umiusi_control/cmd/indicator_led.hpp"
#include "sinsei_umiusi_control/util/gpio_interface.hpp"

namespace sinsei_umiusi_control::hardware_model {

class IndicatorLedModel {
  private:
    std::unique_ptr<util::GpioInterface> gpio;
    util::GpioPin led_pin;

  public:
    IndicatorLedModel(std::unique_ptr<util::GpioInterface> gpio, util::GpioPin led_pin);
    auto on_init() -> tl::expected<void, std::string>;
    auto on_read() -> tl::expected<void, std::string>;
    auto on_write(sinsei_umiusi_control::cmd::indicator_led::Enabled & enabled)
        -> tl::expected<void, std::string>;
};

}  // namespace sinsei_umiusi_control::hardware_model

#endif  // SINSEI_UMIUSI_CONTROL_hardware_model_INDICATOR_LED_MODEL_HPP
#ifndef SINSEI_UMIUSI_CONTROL_hardware_model_INDICATOR_LED_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_hardware_model_INDICATOR_LED_MODEL_HPP

#include <memory>

#include "sinsei_umiusi_control/cmd/indicator_led.hpp"
#include "sinsei_umiusi_control/util/gpio.hpp"

namespace suc = sinsei_umiusi_control;

namespace sinsei_umiusi_control::hardware_model {

class IndicatorLedModel {
  private:
    std::unique_ptr<util::Gpio> gpio;

  public:
    IndicatorLedModel(std::unique_ptr<util::Gpio> gpio);
    auto on_read() -> void;
    auto on_write(bool enabled) -> void;
};

}  // namespace sinsei_umiusi_control::hardware_model

#endif  // SINSEI_UMIUSI_CONTROL_hardware_model_INDICATOR_LED_MODEL_HPP
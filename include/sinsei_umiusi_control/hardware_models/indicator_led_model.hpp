#ifndef SINSEI_UMIUSI_CONTROL_HARDWARE_MODELS_INDICATOR_LED_MODEL_HPP
#define SINSEI_UMIUSI_CONTROL_HARDWARE_MODELS_INDICATOR_LED_MODEL_HPP

#include <memory>

#include "sinsei_umiusi_control/cmd/indicator_led.hpp"
#include "sinsei_umiusi_control/util/gpio.hpp"

namespace suc = sinsei_umiusi_control;

namespace sinsei_umiusi_control::hardware_models {

class IndicatorLedModel {
  private:
    std::shared_ptr<util::GpioWrapper> gpio;

  public:
    IndicatorLedModel(std::shared_ptr<util::GpioWrapper> gpio);
    auto on_read() -> void;
    auto on_write(bool enabled) -> void;
};

}  // namespace sinsei_umiusi_control::hardware_models

#endif  // SINSEI_UMIUSI_CONTROL_HARDWARE_MODELS_INDICATOR_LED_MODEL_HPP
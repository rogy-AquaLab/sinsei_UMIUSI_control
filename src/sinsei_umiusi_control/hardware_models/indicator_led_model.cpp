#include "sinsei_umiusi_control/hardware_models/indicator_led_model.hpp"

#include <memory>

#include "sinsei_umiusi_control/util/gpio.hpp"

namespace suc = sinsei_umiusi_control;
namespace suchm = suc::hardware_models;

suchm::IndicatorLedModel::IndicatorLedModel(std::shared_ptr<util::GpioWrapper> gpio)
: gpio(std::move(gpio)) {}

auto suchm::IndicatorLedModel::on_read() -> void {}

auto suchm::IndicatorLedModel::on_write(bool enabled) -> void {
    this->gpio->write_digital(enabled);
}
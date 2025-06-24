#include "sinsei_umiusi_control/hardware_model/indicator_led_model.hpp"

#include <memory>

#include "sinsei_umiusi_control/util/gpio.hpp"

namespace suc = sinsei_umiusi_control;
namespace suchm = suc::hardware_model;

suchm::IndicatorLedModel::IndicatorLedModel(std::unique_ptr<suc::util::GpioWrapper> gpio)
: gpio(std::move(gpio)) {}

auto suchm::IndicatorLedModel::on_read() -> void {}

auto suchm::IndicatorLedModel::on_write(bool enabled) -> void {
    this->gpio->write_digital(enabled);
}
#include "sinsei_umiusi_control/hardware_model/indicator_led_model.hpp"

namespace suc = sinsei_umiusi_control;
namespace suchm = suc::hardware_model;

suchm::IndicatorLedModel::IndicatorLedModel(std::unique_ptr<suc::util::Gpio> gpio)
: gpio(std::move(gpio)) {}

auto suchm::IndicatorLedModel::on_read() -> void {}

auto suchm::IndicatorLedModel::on_write(
    sinsei_umiusi_control::cmd::indicator_led::Enabled & enabled) -> void {
    this->gpio->write_digital(enabled.value);
}
#include "sinsei_umiusi_control/hardware_model/indicator_led_model.hpp"

#include <rcpputils/tl_expected/expected.hpp>

#include "sinsei_umiusi_control/util/gpio_interface.hpp"

namespace suc = sinsei_umiusi_control;
namespace suchm = suc::hardware_model;

suchm::IndicatorLedModel::IndicatorLedModel(
    std::unique_ptr<suc::util::GpioInterface> gpio, util::GpioPin led_pin)
: gpio(std::move(gpio)), led_pin(led_pin) {}

auto suchm::IndicatorLedModel::on_init() -> tl::expected<void, std::string> {
    return this->gpio->set_mode_output({this->led_pin}).map_error(util::gpio_error_to_string);
}

auto suchm::IndicatorLedModel::on_read() -> tl::expected<void, std::string> { return {}; }

auto suchm::IndicatorLedModel::on_write(sinsei_umiusi_control::cmd::indicator_led::Enabled &
                                            enabled) -> tl::expected<void, std::string> {
    return this->gpio->write_digital(this->led_pin, enabled.value)
        .map_error(util::gpio_error_to_string);
}
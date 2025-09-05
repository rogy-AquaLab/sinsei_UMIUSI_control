#include "sinsei_umiusi_control/hardware_model/indicator_led_model.hpp"

#include <rcpputils/tl_expected/expected.hpp>

#include "sinsei_umiusi_control/hardware_model/interface/gpio.hpp"

using namespace sinsei_umiusi_control::hardware_model;

IndicatorLedModel::IndicatorLedModel(
    std::unique_ptr<interface::Gpio> gpio, interface::Gpio::Pin led_pin)
: gpio(std::move(gpio)), led_pin(led_pin) {}

auto IndicatorLedModel::on_init() -> tl::expected<void, std::string> {
    return this->gpio->set_mode_output({this->led_pin}).map_error(interface::gpio_error_to_string);
}

auto IndicatorLedModel::on_read() const -> tl::expected<void, std::string> { return {}; }

auto IndicatorLedModel::on_write(sinsei_umiusi_control::cmd::indicator_led::Enabled && enabled)
    -> tl::expected<void, std::string> {
    return this->gpio->write_digital(this->led_pin, std::move(enabled.value))
        .map_error(interface::gpio_error_to_string);
}

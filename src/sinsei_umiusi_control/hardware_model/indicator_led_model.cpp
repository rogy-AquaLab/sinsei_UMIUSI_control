#include "sinsei_umiusi_control/hardware_model/indicator_led_model.hpp"

#include <rcpputils/tl_expected/expected.hpp>
#include <utility>

#include "sinsei_umiusi_control/hardware_model/interface/gpio.hpp"

using namespace sinsei_umiusi_control::hardware_model;

IndicatorLedModel::IndicatorLedModel(
    std::unique_ptr<interface::GpioChip> gpio, interface::GpioOffset led_line_offset)
: gpio(std::move(gpio)), led_line_offset(led_line_offset) {}

auto IndicatorLedModel::on_init() -> tl::expected<void, std::string> {
    auto request = interface::GpioOutputRequest{};
    request.offsets = {this->led_line_offset};
    request.initial_values = {interface::GpioValue::Inactive};
    request.consumer = "sinsei_umiusi_control::IndicatorLedModel";

    auto gpio_request = this->gpio->request_outputs(std::move(request));
    if (!gpio_request) {
        return tl::make_unexpected(gpio_request.error());
    }

    this->gpio_request = std::move(gpio_request.value());
    return {};
}

auto IndicatorLedModel::on_read() const -> tl::expected<void, std::string> { return {}; }

auto IndicatorLedModel::on_write(sinsei_umiusi_control::cmd::indicator_led::Enabled && enabled)
    -> tl::expected<void, std::string> {
    if (!this->gpio_request) {
        return tl::make_unexpected("GPIO lines are not initialized");
    }

    return this->gpio_request->set_values({interface::to_gpio_value(enabled.value)});
}

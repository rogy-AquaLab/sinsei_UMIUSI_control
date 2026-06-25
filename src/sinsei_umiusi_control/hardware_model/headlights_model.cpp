#include "sinsei_umiusi_control/hardware_model/headlights_model.hpp"

#include <rcpputils/tl_expected/expected.hpp>
#include <utility>

#include "sinsei_umiusi_control/hardware_model/interface/gpio.hpp"

using namespace sinsei_umiusi_control::hardware_model;

HeadlightsModel::HeadlightsModel(
    std::unique_ptr<interface::Gpio> gpio,
    interface::GpioOffset high_beam_line_offset,  // NOLINT(bugprone-*) TODO: いずれ修正したい
    interface::GpioOffset low_beam_line_offset, interface::GpioOffset ir_line_offset)
: gpio(std::move(gpio)),
  high_beam_line_offset(high_beam_line_offset),
  low_beam_line_offset(low_beam_line_offset),
  ir_line_offset(ir_line_offset) {}

auto HeadlightsModel::on_init() -> tl::expected<void, std::string> {
    auto request = interface::GpioOutputRequest{};
    request.offsets = {
        this->high_beam_line_offset,
        this->low_beam_line_offset,
        this->ir_line_offset,
    };
    request.initial_values = {
        interface::GpioValue::Inactive,
        interface::GpioValue::Inactive,
        interface::GpioValue::Inactive,
    };
    request.consumer = "sinsei_umiusi_control::HeadlightsModel";

    auto gpio_request = this->gpio->request_outputs(std::move(request));
    if (!gpio_request) {
        return tl::make_unexpected(gpio_request.error());
    }

    this->gpio_request = std::move(gpio_request.value());
    return {};
}

auto HeadlightsModel::on_read() const -> tl::expected<void, std::string> { return {}; }

auto HeadlightsModel::on_write(
    cmd::headlights::HighBeamEnabled && high_beam_enabled,
    cmd::headlights::LowBeamEnabled && low_beam_enabled,
    cmd::headlights::IrEnabled && ir_enabled) -> tl::expected<void, std::string> {
    if (!this->gpio_request) {
        return tl::make_unexpected("GPIO lines are not initialized");
    }

    return this->gpio_request->set_values({
        interface::to_gpio_value(high_beam_enabled.value),
        interface::to_gpio_value(low_beam_enabled.value),
        interface::to_gpio_value(ir_enabled.value),
    });
}

#include "sinsei_umiusi_control/hardware_model/headlights_model.hpp"

#include <rcpputils/tl_expected/expected.hpp>

#include "sinsei_umiusi_control/hardware_model/interface/gpio.hpp"

using namespace sinsei_umiusi_control::hardware_model;

HeadlightsModel::HeadlightsModel(
    std::unique_ptr<interface::Gpio> gpio,
    interface::Gpio::Pin high_beam_pin,  // NOLINT(bugprone-*) TODO: いずれ修正したい
    interface::Gpio::Pin low_beam_pin, interface::Gpio::Pin ir_pin)
: gpio(std::move(gpio)), high_beam_pin(high_beam_pin), low_beam_pin(low_beam_pin), ir_pin(ir_pin) {}

auto HeadlightsModel::on_init() -> tl::expected<void, std::string> {
    return this->gpio->set_mode_output({this->high_beam_pin, this->low_beam_pin, this->ir_pin})
        .map_error(interface::gpio_error_to_string);
}

auto HeadlightsModel::on_read() const -> tl::expected<void, std::string> { return {}; }

auto HeadlightsModel::on_write(
    cmd::headlights::HighBeamEnabled && high_beam_enabled,
    cmd::headlights::LowBeamEnabled && low_beam_enabled,
    cmd::headlights::IrEnabled && ir_enabled) -> tl::expected<void, std::string> {
    const auto res_high =
        this->gpio
            ->write_digital(std::move(this->high_beam_pin), std::move(high_beam_enabled.value))
            .map_error(interface::gpio_error_to_string);
    const auto res_low =
        this->gpio->write_digital(this->low_beam_pin, std::move(low_beam_enabled.value))
            .map_error(interface::gpio_error_to_string);
    const auto res_ir = this->gpio->write_digital(this->ir_pin, std::move(ir_enabled.value))
                            .map_error(interface::gpio_error_to_string);

    if (res_high && res_low && res_ir) {
        return {};
    }

    const auto & msg_high = res_high ? "Success" : res_high.error();
    const auto & msg_low = res_low ? "Success" : res_low.error();
    const auto & msg_ir = res_ir ? "Success" : res_ir.error();

    // すべてのエラーをまとめて返す
    return tl::unexpected(
        "Failed to write to GPIO pins:\n    High Beam: " + msg_high + "\n    Low Beam: " + msg_low +
        "\n    IR: " + msg_ir);
}

#include "sinsei_umiusi_control/hardware_model/headlights_model.hpp"

#include <rcpputils/tl_expected/expected.hpp>

#include "sinsei_umiusi_control/hardware_model/interface/gpio_interface.hpp"

namespace suc = sinsei_umiusi_control;
namespace suchm = suc::hardware_model;

suchm::HeadlightsModel::HeadlightsModel(
    std::unique_ptr<suchm::interface::GpioInterface> gpio,
    suchm::interface::GpioPin high_beam_pin,  // NOLINT(bugprone-*) TODO: いずれ修正したい
    suchm::interface::GpioPin low_beam_pin, suchm::interface::GpioPin ir_pin)
: gpio(std::move(gpio)), high_beam_pin(high_beam_pin), low_beam_pin(low_beam_pin), ir_pin(ir_pin) {}

auto suchm::HeadlightsModel::on_init() -> tl::expected<void, std::string> {
    return this->gpio->set_mode_output({this->high_beam_pin, this->low_beam_pin, this->ir_pin})
        .map_error(suchm::interface::gpio_error_to_string);
}

auto suchm::HeadlightsModel::on_read() -> tl::expected<void, std::string> { return {}; }

auto suchm::HeadlightsModel::on_write(
    sinsei_umiusi_control::cmd::headlights::HighBeamEnabled & high_beam_enabled,
    sinsei_umiusi_control::cmd::headlights::LowBeamEnabled & low_beam_enabled,
    sinsei_umiusi_control::cmd::headlights::IrEnabled & ir_enabled)
    -> tl::expected<void, std::string> {
    auto res_high = this->gpio->write_digital(this->high_beam_pin, high_beam_enabled.value)
                        .map_error(suchm::interface::gpio_error_to_string);
    auto res_low = this->gpio->write_digital(this->low_beam_pin, low_beam_enabled.value)
                       .map_error(suchm::interface::gpio_error_to_string);
    auto res_ir = this->gpio->write_digital(this->ir_pin, ir_enabled.value)
                      .map_error(suchm::interface::gpio_error_to_string);

    if (res_high && res_low && res_ir) {
        return {};
    }

    auto msg_high = res_high ? "Success" : res_high.error();
    auto msg_low = res_low ? "Success" : res_low.error();
    auto msg_ir = res_ir ? "Success" : res_ir.error();

    // すべてのエラーをまとめて返す
    auto msg = "Failed to write to GPIO pins:\n    High Beam: " + msg_high +
               "\n    Low Beam: " + msg_low + "\n    IR: " + msg_ir;

    return tl::unexpected(msg);
}

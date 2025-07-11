#include "sinsei_umiusi_control/hardware_model/headlights_model.hpp"

namespace suc = sinsei_umiusi_control;
namespace suchm = suc::hardware_model;

suchm::HeadlightsModel::HeadlightsModel(
    std::unique_ptr<suc::util::GpioInterface> gpio,
    uint8_t high_beam_pin,  // NOLINT(bugprone-easily-swappable-parameters) TODO: いずれ修正したい
    uint8_t low_beam_pin, uint8_t ir_pin)
: gpio(std::move(gpio)), high_beam_pin(high_beam_pin), low_beam_pin(low_beam_pin), ir_pin(ir_pin) {}

auto suchm::HeadlightsModel::on_init() -> tl::expected<void, std::string> {
    return this->gpio->set_mode_output({this->high_beam_pin, this->low_beam_pin, this->ir_pin})
        .map_error([](const auto & e) {
            switch (e) {
                case util::GpioError::BadGpio:
                    return std::string("Bad GPIO pin specified for Headlights");
                case util::GpioError::NotPermitted:
                    return std::string("Not permitted to set GPIO pin mode for Headlights");
                default:
                    return std::string(
                        "Unknown error occurred while setting GPIO pin mode for Headlights");
            }
        });
}

auto suchm::HeadlightsModel::on_read() -> void {}

auto suchm::HeadlightsModel::on_write(
    sinsei_umiusi_control::cmd::headlights::HighBeamEnabled & high_beam_enabled,
    sinsei_umiusi_control::cmd::headlights::LowBeamEnabled & low_beam_enabled,
    sinsei_umiusi_control::cmd::headlights::IrEnabled & ir_enabled) -> void {
    this->gpio->write_digital(this->high_beam_pin, high_beam_enabled.value);
    this->gpio->write_digital(this->low_beam_pin, low_beam_enabled.value);
    this->gpio->write_digital(this->ir_pin, ir_enabled.value);
}
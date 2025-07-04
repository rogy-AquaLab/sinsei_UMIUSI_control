#include "sinsei_umiusi_control/hardware_model/headlights_model.hpp"

namespace suc = sinsei_umiusi_control;
namespace suchm = suc::hardware_model;

suchm::HeadlightsModel::HeadlightsModel(
    std::unique_ptr<suc::util::Gpio> high_beam_gpio, std::unique_ptr<suc::util::Gpio> low_beam_gpio,
    std::unique_ptr<suc::util::Gpio> ir_gpio)
: high_beam_gpio(std::move(high_beam_gpio)),
  low_beam_gpio(std::move(low_beam_gpio)),
  ir_gpio(std::move(ir_gpio)) {}

auto suchm::HeadlightsModel::on_read() -> void {}

auto suchm::HeadlightsModel::on_write(
    sinsei_umiusi_control::cmd::headlights::HighBeamEnabled & high_beam_enabled,
    sinsei_umiusi_control::cmd::headlights::LowBeamEnabled & low_beam_enabled,
    sinsei_umiusi_control::cmd::headlights::IrEnabled & ir_enabled) -> void {
    this->high_beam_gpio->write_digital(high_beam_enabled.value);
    this->low_beam_gpio->write_digital(low_beam_enabled.value);
    this->ir_gpio->write_digital(ir_enabled.value);
}
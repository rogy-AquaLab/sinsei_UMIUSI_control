#include "sinsei_umiusi_control/hardware_model/headlights_model.hpp"

#include <rcpputils/tl_expected/expected.hpp>

#include "sinsei_umiusi_control/util/gpio_interface.hpp"

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

auto suchm::HeadlightsModel::on_read() -> tl::expected<void, std::string> { return {}; }

auto suchm::HeadlightsModel::on_write(
    sinsei_umiusi_control::cmd::headlights::HighBeamEnabled & high_beam_enabled,
    sinsei_umiusi_control::cmd::headlights::LowBeamEnabled & low_beam_enabled,
    sinsei_umiusi_control::cmd::headlights::IrEnabled & ir_enabled)
    -> tl::expected<void, std::string> {
    auto res_high = this->gpio->write_digital(this->high_beam_pin, high_beam_enabled.value);
    auto res_low = this->gpio->write_digital(this->low_beam_pin, low_beam_enabled.value);
    auto res_ir = this->gpio->write_digital(this->ir_pin, ir_enabled.value);

    if (res_high && res_low && res_ir) {
        return {};
    }

    // エラーメッセージを生成する変換を定義しておく
    auto res_to_msg = [](const auto & res) {
        if (res.has_value()) {
            return std::string("Success");
        }
        switch (res.error()) {
            case util::GpioError::BadGpio:
                return std::string("Error: Bad GPIO pin specified");
            case util::GpioError::BadLevel:
                return std::string("Error: Bad level specified");
            case util::GpioError::NotPermitted:
                return std::string("Error: Not permitted to write to GPIO pin");
            default:
                return std::string("Error: Unknown error occurred while writing to GPIO pin");
        }
    };
    auto msg_high = res_to_msg(res_high);
    auto msg_low = res_to_msg(res_low);
    auto msg_ir = res_to_msg(res_ir);

    // すべてのエラーをまとめて返す
    auto msg = "Failed to write to GPIO pins:\nHigh Beam: " + msg_high + "\n" +
               "Low Beam: " + msg_low + "\n" + "IR: " + msg_ir;

    return tl::unexpected(msg);
}

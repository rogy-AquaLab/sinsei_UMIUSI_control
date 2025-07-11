#include "sinsei_umiusi_control/hardware_model/indicator_led_model.hpp"

#include <rcpputils/tl_expected/expected.hpp>

#include "sinsei_umiusi_control/util/gpio_interface.hpp"

namespace suc = sinsei_umiusi_control;
namespace suchm = suc::hardware_model;

suchm::IndicatorLedModel::IndicatorLedModel(
    std::unique_ptr<suc::util::GpioInterface> gpio, uint8_t led_pin)
: gpio(std::move(gpio)), led_pin(led_pin) {}

auto suchm::IndicatorLedModel::on_init() -> tl::expected<void, std::string> {
    return this->gpio->set_mode_output({this->led_pin}).map_error([](const auto & e) {
        switch (e) {
            case util::GpioError::BadGpio:
                return std::string("Bad GPIO pin specified for Indicator LED");
            case util::GpioError::NotPermitted:
                return std::string("Not permitted to set GPIO pin mode for Indicator LED");
            default:
                return std::string(
                    "Unknown error occurred while setting GPIO pin mode for Indicator LED");
        }
    });
}

auto suchm::IndicatorLedModel::on_read() -> tl::expected<void, std::string> { return {}; }

auto suchm::IndicatorLedModel::on_write(sinsei_umiusi_control::cmd::indicator_led::Enabled &
                                            enabled) -> tl::expected<void, std::string> {
    return this->gpio->write_digital(this->led_pin, enabled.value).map_error([](const auto & e) {
        switch (e) {
            case util::GpioError::BadGpio:
                return std::string("Bad GPIO pin specified for Indicator LED");
            case util::GpioError::BadLevel:
                return std::string("Bad level specified for Indicator LED");
            case util::GpioError::NotPermitted:
                return std::string("Not permitted to write to GPIO pin for Indicator LED");
            default:
                return std::string(
                    "Unknown error occurred while writing to GPIO pin for Indicator LED");
        }
    });
}
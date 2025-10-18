#include "sinsei_umiusi_control/hardware_model/thruster_direct/esc_direct_model.hpp"

using namespace sinsei_umiusi_control::hardware_model::thruster_direct;

EscDirectModel::EscDirectModel(std::unique_ptr<interface::Gpio> gpio, interface::Gpio::Pin esc_pin)
: gpio(std::move(gpio)), esc_pin(esc_pin) {}

auto EscDirectModel::on_init() -> tl::expected<void, std::string> {
    return this->gpio->set_mode_output({this->esc_pin}).map_error(interface::gpio_error_to_string);
}

auto EscDirectModel::on_read() const -> tl::expected<void, std::string> { return {}; }

auto EscDirectModel::on_write(
    sinsei_umiusi_control::cmd::thruster::esc::Enabled && enabled,
    sinsei_umiusi_control::cmd::thruster::esc::DutyCycle && duty_cycle)
    -> tl::expected<void, std::string> {
    return this->gpio
        ->write_pwm_duty(this->esc_pin, enabled.value ? std::move(duty_cycle.value) : 0.0)
        .map_error(interface::gpio_error_to_string);
}

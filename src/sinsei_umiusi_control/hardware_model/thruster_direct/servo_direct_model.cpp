#include "sinsei_umiusi_control/hardware_model/thruster_direct/servo_direct_model.hpp"

using namespace sinsei_umiusi_control::hardware_model::thruster_direct;

ServoDirectModel::ServoDirectModel(
    std::unique_ptr<interface::Gpio> gpio, interface::Gpio::Pin servo_pin)
: gpio(std::move(gpio)), servo_pin(servo_pin) {}

auto ServoDirectModel::on_init() -> tl::expected<void, std::string> {
    return this->gpio->set_mode_output({this->servo_pin})
        .map_error(interface::gpio_error_to_string);
}

auto ServoDirectModel::on_read() const -> tl::expected<void, std::string> { return {}; }

auto ServoDirectModel::on_write(
    sinsei_umiusi_control::cmd::thruster::servo::Enabled && enabled,
    sinsei_umiusi_control::cmd::thruster::servo::Angle && angle)
    -> tl::expected<void, std::string> {
    return this->gpio
        ->write_servo_angle(this->servo_pin, std::move(enabled.value), std::move(angle.value))
        .map_error(interface::gpio_error_to_string);
}

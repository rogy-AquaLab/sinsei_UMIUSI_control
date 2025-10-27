#include "sinsei_umiusi_control/hardware_model/thruster_direct/servo_direct_model.hpp"

#include "sinsei_umiusi_control/hardware_model/interface/gpio.hpp"

using namespace sinsei_umiusi_control::hardware_model::thruster_direct;

ServoDirectModel::ServoDirectModel(
    std::unique_ptr<interface::Gpio> gpio,
    interface::Gpio::Pin servo_pin,  // NOLINT(bugprone-*) TODO: いずれ修正したい
    interface::Gpio::PulseWidth min_pulse_width, interface::Gpio::PulseWidth max_pulse_width)
: gpio(std::move(gpio)),
  servo_pin(servo_pin),
  min_pulse_width(min_pulse_width),
  max_pulse_width(max_pulse_width) {}

auto ServoDirectModel::on_init() -> tl::expected<void, std::string> {
    return this->gpio->set_mode_output({this->servo_pin})
        .map_error(interface::gpio_error_to_string);
}

auto ServoDirectModel::on_read() const -> tl::expected<void, std::string> { return {}; }

auto ServoDirectModel::on_write(
    sinsei_umiusi_control::cmd::thruster::servo::Enabled && enabled,
    sinsei_umiusi_control::cmd::thruster::servo::Angle && angle)
    -> tl::expected<void, std::string> {
    // 角度をパルス幅に変換
    const auto min = this->min_pulse_width;
    const auto max = this->max_pulse_width;
    auto pulsewidth = enabled.value ? min + (max - min) * (angle.value + 90.0) / 180.0 : 0;
    return this->gpio
        ->write_pwm_pulsewidth(
            this->servo_pin, static_cast<interface::Gpio::PulseWidth>(pulsewidth))
        .map_error(interface::gpio_error_to_string);
}

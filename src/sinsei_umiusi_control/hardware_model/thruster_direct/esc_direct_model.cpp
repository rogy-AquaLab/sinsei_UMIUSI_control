#include "sinsei_umiusi_control/hardware_model/thruster_direct/esc_direct_model.hpp"

using namespace sinsei_umiusi_control::hardware_model::thruster_direct;

EscDirectModel::EscDirectModel(
    std::unique_ptr<interface::Gpio> gpio,
    interface::Gpio::Pin esc_pin,  // NOLINT(bugprone-*) TODO: いずれ修正したい
    interface::Gpio::PulseWidth center_pulse_width,
    interface::Gpio::PulseWidth negative_pulse_width_radius,
    interface::Gpio::PulseWidth positive_pulse_width_radius)
: gpio(std::move(gpio)),
  esc_pin(esc_pin),
  center_pulse_width(center_pulse_width),
  negative_pulse_width_radius(negative_pulse_width_radius),
  positive_pulse_width_radius(positive_pulse_width_radius) {}

auto EscDirectModel::on_init() -> tl::expected<void, std::string> {
    return this->gpio->set_mode_output({this->esc_pin}).map_error(interface::gpio_error_to_string);
}

auto EscDirectModel::on_read() const -> tl::expected<void, std::string> { return {}; }

auto EscDirectModel::on_write(
    sinsei_umiusi_control::cmd::thruster::esc::Enabled && enabled,
    sinsei_umiusi_control::cmd::thruster::esc::DutyCycle && duty_cycle)
    -> tl::expected<void, std::string> {
    // Duty比をパルス幅に変換
    const auto center = this->center_pulse_width;
    const auto negative_radius = this->negative_pulse_width_radius;
    const auto positive_radius = this->positive_pulse_width_radius;
    auto pulsewidth = enabled.value
                          ? center + (duty_cycle.value < 0.0 ? negative_radius * duty_cycle.value
                                                             : positive_radius * duty_cycle.value)
                          : 0;
    return this->gpio
        ->write_servo_pulsewidth(
            this->esc_pin, static_cast<interface::Gpio::PulseWidth>(pulsewidth))
        .map_error(interface::gpio_error_to_string);
}

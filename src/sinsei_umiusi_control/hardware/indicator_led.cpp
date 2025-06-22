#include "sinsei_umiusi_control/hardware/indicator_led.hpp"

#include <pigpiod_if2.h>

namespace suchw = sinsei_umiusi_control::hardware;
namespace hif = hardware_interface;
namespace rlc = rclcpp_lifecycle;

auto suchw::IndicatorLed::on_init(const hif::HardwareInfo & info) -> hif::CallbackReturn {
    this->hif::SystemInterface::on_init(info);

    this->pi = pigpio_start(NULL, NULL);
    set_mode(pi, INDICATOR_LED_GPIO, PI_OUTPUT);

    return hif::CallbackReturn::SUCCESS;
}

auto suchw::IndicatorLed::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*preiod*/)
    -> hif::return_type {
    return hif::return_type::OK;
}

auto suchw::IndicatorLed::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    -> hif::return_type {
    double enabled_raw = get_command("indicator_led/indicator_led/enabled");
    auto enabled =
        *reinterpret_cast<sinsei_umiusi_control::cmd::indicator_led::Enabled *>(&enabled_raw);
    gpio_write(pi, INDICATOR_LED_GPIO, enabled.value ? 1 : 0);

    return hif::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::hardware::IndicatorLed, hardware_interface::SystemInterface)
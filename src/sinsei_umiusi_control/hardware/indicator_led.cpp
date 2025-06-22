#include "sinsei_umiusi_control/hardware/indicator_led.hpp"

#include <pigpio.h>

namespace suchw = sinsei_umiusi_control::hardware;
namespace hif = hardware_interface;
namespace rlc = rclcpp_lifecycle;

auto suchw::IndicatorLed::on_init(const hif::HardwareInfo & info) -> hif::CallbackReturn {
    this->hif::SystemInterface::on_init(info);

    RCLCPP_INFO(get_logger(), "Initializing IndicatorLed hardware interface...");
    auto status = gpioInitialise();
    this->gpio_initialized = status >= 0;

    gpioSetMode(INDICATOR_LED_GPIO, PI_OUTPUT);

    return hif::CallbackReturn::SUCCESS;
}

auto suchw::IndicatorLed::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*preiod*/)
    -> hif::return_type {
    return hif::return_type::OK;
}

auto suchw::IndicatorLed::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    -> hif::return_type {
    if (!this->gpio_initialized) {
        // GPIOが使えない場合（Docker内など）は何もしない
        return hif::return_type::OK;
    }

    double enabled_raw = get_command("indicator_led/indicator_led/enabled");
    auto enabled =
        *reinterpret_cast<sinsei_umiusi_control::cmd::indicator_led::Enabled *>(&enabled_raw);
    gpioWrite(INDICATOR_LED_GPIO, enabled.value ? 1 : 0);

    return hif::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::hardware::IndicatorLed, hardware_interface::SystemInterface)
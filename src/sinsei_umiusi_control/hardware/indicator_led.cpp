#include "sinsei_umiusi_control/hardware/indicator_led.hpp"

namespace suchw = sinsei_umiusi_control::hardware;
namespace hif = hardware_interface;
namespace rlc = rclcpp_lifecycle;

auto suchw::IndicatorLed::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*preiod*/)
    -> hif::return_type {
    return hif::return_type::OK;
}

auto suchw::IndicatorLed::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    -> hif::return_type {
    // TODO: 以下はデバッグ用。実際はLEDの点灯処理に変更する
    double enabled_raw = get_command("indicator_led/indicator_led/enabled");
    auto enabled =
        *reinterpret_cast<sinsei_umiusi_control::cmd::indicator_led::Enabled *>(&enabled_raw);

    RCLCPP_INFO(get_logger(), enabled.value ? "true" : "false");
    return hif::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::hardware::IndicatorLed, hardware_interface::SystemInterface)
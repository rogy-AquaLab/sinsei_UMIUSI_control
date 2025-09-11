#include "sinsei_umiusi_control/hardware/thruster_direct/servo_direct.hpp"

using namespace sinsei_umiusi_control::hardware;

auto thruster_direct::ServoDirect::read(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*preiod*/) -> hardware_interface::return_type {
    return hardware_interface::return_type::OK;
}

auto thruster_direct::ServoDirect::write(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/) -> hardware_interface::return_type {
    return hardware_interface::return_type::OK;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    sinsei_umiusi_control::hardware::thruster_direct::ServoDirect,
    hardware_interface::SystemInterface)
